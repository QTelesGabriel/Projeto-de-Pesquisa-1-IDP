# Relatório Técnico: Fundamentação Matemática, Física e Algorítmica da Estabilidade de Voo do VANT

**Projeto:** Sistema Autônomo de Captura de Objetos por VANTs via Servovisão (*Autonomous Object Capture System by UAVs Using Visual Servoing*)  
**Autor:** Gabriel Queiroz Teles  
**Instituição:** IDP — Ciência da Computação  
**Objetivo do Documento:** Explicar detalhadamente, com rigor físico-matemático e correlação com o código-fonte, todos os cálculos e estratégias que tornam o voo do drone estável durante as fases de busca, aproximação, alinhamento de atitude/yaw e captura milimétrica.

---

## Sumário Executivo: Por que o Drone Não Cai Nem Oscila?

Para que um drone quadricóptero consiga voar de forma estável, pairar (*hover*) e perseguir um alvo no solo sem desestabilizar ou cair, o sistema foi estruturado em **4 camadas interdependentes de estabilização**:

1. **Estabilidade Dinâmica de Baixo Nível (Física e Atitude no ArduPilot SITL):** Resolução contínua das equações de Newton-Euler a ~400 Hz para manter o drone nivelado e controlar a inclinação necessária para transladar.
2. **Estabilidade Métrica Espacial (PBVS e Modelo Pinhole):** Conversão de coordenadas bidimensionais de imagem (pixels) para metros reais no espaço tridimensional (3D), eliminando a ambiguidade causada pela variação de altitude.
3. **Estabilidade de Sinal e Filtragem Estocástica (Filtros de Kalman - EKF):** Amortecimento de ruídos de detecção da rede neural (YOLO), supressão de *jitter* e predição inercial durante eventuais quedas de quadros (*dropped frames*).
4. **Estabilidade Cinemática de Alto Nível (Controle em Cascata PD e Histerese Ótica):** Uso de controladores Proporcional-Derivativo (PD) com supressão do termo Integral para evitar *overshoot*, histerese espacial para retenção contínua do alvo no campo de visão (FOV), e descida final cega (*blind drop*) para furar a turbulência do efeito solo.

---

## 1. A Dinâmica Física do Quadricóptero (Modelo Newton-Euler)

### 1.1. O Desafio da Subatuação (Under-Actuation)
Um quadricóptero é um corpo rígido no espaço tridimensional com **6 Graus de Liberdade (6 DoF)**:
- 3 translações: posição Cartesiana $\mathbf{p} = [x, y, z]^T \in \mathbb{R}^3$.
- 3 rotações: ângulos de Euler de atitude $\boldsymbol{\eta} = [\phi, \theta, \psi]^T$ (Roll/Rolagem, Pitch/Arfagem, Yaw/Guinada).

No entanto, o robô possui apenas **4 entradas de controle independentes**, que correspondem à velocidade angular das 4 hélices ($\omega_1, \omega_2, \omega_3, \omega_4$). As forças e momentos gerados são:
- **Empuxo Total ($T$ ou $U_1$):** $T = \sum_{i=1}^4 k_t \omega_i^2$, onde $k_t$ é a constante aerodinâmica de empuxo.
- **Momento de Roll ($\tau_\phi$ ou $U_2$):** Diferença de empuxo entre os motores laterais.
- **Momento de Pitch ($\tau_\theta$ ou $U_3$):** Diferença de empuxo entre os motores frontais e traseiros.
- **Momento de Yaw ($\tau_\psi$ ou $U_4$):** Reação do torque de arrasto das hélices ($\tau_i = k_d \omega_i^2$), resultante da diferença entre os pares que giram em sentido horário (CW) e anti-horário (CCW).

Como o vetor de empuxo total $\mathbf{T}$ é gerado **exclusivamente na direção do eixo vertical do corpo do drone ($Z_B$)**, o quadricóptero é fisicamente incapaz de gerar forças laterais diretas sem alterar sua atitude.

### 1.2. Equações do Movimento Translacional e Rotacional
Pelo formalismo de Newton-Euler no referencial inercial do mundo ($F_0$) e no referencial móvel do corpo ($F_B$):

$$\mathbf{m} \ddot{\mathbf{p}} = \begin{bmatrix} 0 \\ 0 \\ mg \end{bmatrix} - \mathbf{R}_{B}^{0} \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} - \mathbf{D} \dot{\mathbf{p}}$$

$$\mathbf{J} \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{J} \boldsymbol{\omega}) = \boldsymbol{\tau} - \boldsymbol{\tau}_{\text{giro}}$$

Onde:
- $m$ é a massa total do drone e $\mathbf{J}$ é o tensor de inércia.
- $\mathbf{D}$ é a matriz de arrasto aerodinâmico.
- $\mathbf{R}_{B}^{0} \in SO(3)$ é a matriz de rotação ortogonal que mapeia o corpo para o mundo:

$$\mathbf{R}_{B}^{0} = \begin{bmatrix} 
\cos\theta\cos\psi & \sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi & \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \\
\cos\theta\sin\psi & \sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi & \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\
-\sin\theta & \sin\phi\cos\theta & \cos\phi\cos\theta
\end{bmatrix}$$

### 1.3. O Princípio Físico da Translação
Para acelerar para frente ($+X$), o drone precisa inclinar o nariz para baixo ($\theta < 0$). Para acelerar para a direita ($+Y$), precisa rolar para a direita ($\phi > 0$).
A aceleração horizontal surge da projeção trigonométrica do empuxo:
$$a_x \approx g \cdot \theta, \quad a_y \approx -g \cdot \phi$$
Simultaneamente, para não perder altitude durante a inclinação, o empuxo vertical total precisa ser aumentado:
$$T = \frac{mg}{\cos\phi \cos\theta}$$

---

## 2. A Arquitetura em Cascata (Malha Interna vs. Malha Externa)

A estabilidade total é garantida pela separação temporal em **duas malhas de controle (Cascata)**:

```
+-----------------------------------------------------------------------------------------+
|                                    MALHA EXTERNA (Python / ROS2 @ ~20 Hz)                |
|                                                                                         |
|  [Câmera Gazebo] ---> [YOLOv11] ---> [Filtro EKF] ---> [Modelo Pinhole (PBVS)]          |
|                                                                |                        |
|                                                         Erro 3D (m)                     |
|                                                                v                        |
|                                                      [Controlador PD]                   |
|                                                                |                        |
|                                                    Velocidades (Vx, Vy, Vz, YawRate)    |
+----------------------------------------------------------------|------------------------+
                                                                 | Protocolo MAVLink
                                                                 v (set_position_target_local_ned)
+----------------------------------------------------------------|------------------------+
|                                    MALHA INTERNA (ArduPilot SITL @ ~400 Hz)              |
|                                                                                         |
|  [Comando Velocidade] ---> [PID Posição/Velocidade] ---> [Ângulos Alvo: Roll, Pitch]    |
|                                                                     |                   |
|                                                                     v                   |
|  [Sensores IMU / Giroscópios] ----------------------------> [PID Atitude/Taxas]         |
|                                                                     |                   |
|                                                                     v                   |
|                                                              [Mixer de Motores]         |
|                                                                     |                   |
|                                                              PWM para ESCs              |
|                                                                     v                   |
|                                                               [4 Motores Brushless]     |
+-----------------------------------------------------------------------------------------+
```

### Por que essa separação garante estabilidade?
1. **Diferença de Escala Temporal (Separação Espectral):** A dinâmica de atitude (Roll/Pitch/Yaw) precisa reagir em milissegundos para rejeitar perturbações do ar. Ela roda no firmware (ArduPilot) a **400 Hz**.
2. **Desacoplamento de Tarefas:** O script em Python não se preocupa em controlar a rotação de cada hélice individual. Ele atua como um navegador de alto nível, emitindo comandos cinemáticos de velocidade linear:
   ```python
   enviar_velocidade(vehicle, velocity_x, velocity_y, velocity_z, yaw_rate)
   ```
   O ArduPilot recebe esses vetores no referencial `MAV_FRAME_BODY_NED` e calcula automaticamente os ângulos de inclinação necessários para atingir essas velocidades de forma suave.

---

## 3. O Cálculo da Posição Métrica 3D: Do Pixel ao Metro (PBVS)

### 3.1. O Problema do Controle em Pixels (IBVS)
Se o controle utilizasse diretamente o erro em pixels ($e_u = u_{alvo} - c_x$):
- A **18 metros de altitude**, um erro de 100 pixels equivale a aproximadamente **1,6 metros físicos no chão**.
- A **1 metro de altitude**, os mesmos 100 pixels equivalem a apenas **9 centímetros**.
Se o ganho de controle fosse constante em pixels, o drone seria extremamente lento em altas altitudes e violento/instável em baixas altitudes, oscilando até a colisão.

### 3.2. A Solução: Modelo de Câmera Pinhole Inverso (PBVS)
Para tornar o erro fisicamente uniforme em qualquer altitude, o código implementa a projeção inversa da geometria de perspectiva:

$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_c} \mathbf{K} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \frac{1}{Z_c} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}$$

Invertendo as equações para encontrar as coordenadas reais no solo ($X_c, Y_c$ em metros):

$$X_m = \frac{(u - c_x) \cdot Z_{\text{rel}}}{f_x}$$

$$Y_m = \frac{(v - c_y) \cdot Z_{\text{rel}}}{f_y}$$

Onde:
- $u, v$: Coordenadas do centro do alvo detectado na imagem (em pixels).
- $c_x, c_y$: Ponto principal / centro óptico da lente (lido via tópico ROS2 `CameraInfo`: $c_x \approx 960$, $c_y \approx 540$).
- $f_x, f_y$: Distâncias focais da lente em pixels ($f_x = f_y \approx 1124.0$).
- $Z_{\text{rel}}$: Distância vertical real entre a lente da câmera e o topo da gaiola:
  $$Z_{\text{rel}} = \max(0.0, \text{altitude}_{\text{barômetro/GPS}} - \text{altura}_{\text{gaiola}})$$
  com $\text{altura}_{\text{gaiola}} = 0.317\text{ m}$ (extraída do modelo CAD/STL).

**Trecho de Código correspondente (`rastreador.py` e `rastreador_fino.py`):**
```python
# Conversão PBVS exata de Pixels para Metros
x_m = (centro_gaiola_x - cx) * altitude / fx
y_m = (centro_gaiola_y - cy) * altitude / fy
```

---

## 4. O Filtro de Kalman Estendido (EKF): Amortecendo o Jitter da IA

### 4.1. Por que a IA Pura Causa Instabilidade?
Modelos de detecção baseados em redes neurais profundas (YOLOv11n) sofrem de:
1. **Jitter de Bounding Box:** Pequenas oscilações de 2 a 5 pixels entre frames consecutivos devido à quantização da imagem e ruído de luz.
2. **Latência de Processamento:** Cada inferência consome ~15 a 30 ms.
3. **Quedas Esporádicas de Detecção:** Oclusões parciais ou desfoque de movimento (*motion blur*) podem fazer a rede perder o alvo por 1 ou mais frames.

Se o controlador reagisse a cada oscilação instantânea da YOLO, os motores sofreriam acelerações bruscas contínuas, degradando a estabilidade de atitude.

### 4.2. Modelagem Matemática do Filtro de Kalman
Para o rastreamento do alvo no plano horizontal, definiu-se o vetor de estado:
$$\mathbf{x}_k = \begin{bmatrix} x & y & v_x & v_y \end{bmatrix}^T$$
A medição fornecida pela visão é a posição Cartesiana estimada pelo Pinhole:
$$\mathbf{z}_k = \begin{bmatrix} z_x & z_y \end{bmatrix}^T$$

#### Passo 1: Predição Física (Modelo Cinemático a Velocidade Constante)
$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}_k \hat{\mathbf{x}}_{k-1|k-1}$$
$$\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}$$

Onde a matriz de transição de estado $\mathbf{F}_k$ para o intervalo $\Delta t$ é:
$$\mathbf{F}_k = \begin{bmatrix} 
1 & 0 & \Delta t & 0 \\ 
0 & 1 & 0 & \Delta t \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 
\end{bmatrix}$$

E a matriz de covariância do ruído de processo $\mathbf{Q}$ foi calibrada no código como:
$$\mathbf{Q} = \begin{bmatrix} 
0.01 & 0 & 0 & 0 \\ 
0 & 0.01 & 0 & 0 \\ 
0 & 0 & 0.1 & 0 \\ 
0 & 0 & 0 & 0.1 
\end{bmatrix}$$
Isso impõe que a posição não pode "teletransportar" abruptamente, forçando continuidade nas trajetórias.

#### Passo 2: Atualização com a Medição da YOLO
Quando a YOLO detecta o alvo, a inovação $\mathbf{y}_k$ e o Ganho de Kalman $\mathbf{K}_k$ são computados:
$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}$$
$$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}$$
$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$
$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$$

Onde a matriz de ruído de medição da YOLO foi configurada como:
$$\mathbf{R} = \begin{bmatrix} 0.5 & 0 \\ 0 & 0.5 \end{bmatrix}$$

#### Comportamento sob Perda Visual (Coast Mode / Predição Pura)
Se a YOLO não detectar o alvo em um determinado frame ($\mathbf{z}_k = \text{None}$), o filtro executa apenas a etapa de **Predição**. A posição estimada continua evoluindo suavemente com base na velocidade inercial $\mathbf{v}$, impedindo solavancos no drone por até **4 segundos de tolerância** (`timeout_limite = 4.0`).

---

## 5. O Controlador Proporcional-Derivativo (PD)

### 5.1. A Lei de Controle Translacional
O cálculo das velocidades lineares enviadas ao piloto automático segue a lei de controle:

$$v_x(t) = K_p \cdot e_x(t) + K_d \cdot \frac{e_x(t) - e_x(t - \Delta t)}{\Delta t}$$

$$v_y(t) = K_p \cdot e_y(t) + K_d \cdot \frac{e_y(t) - e_y(t - \Delta t)}{\Delta t}$$

Onde $e_x, e_y$ são as distâncias em metros entre o centro da câmera e o alvo no solo.

### 5.2. Por que NÃO usar o Termo Integral ($K_i$)?
Em controle clássico de servovisão com processamento embarcado, a câmera introduz um atraso temporal puro (latência de transporte $\tau \approx 50$ ms).
- Um integrador $\int e(t) dt$ acumula erro durante esse período de atraso (*integrator windup*).
- Quando o drone atinge o centro do alvo ($e = 0$), o integrador ainda possui valor acumulado residual, forçando o drone a continuar se movendo.
- Isso gera **ultrapassagem de trajetória (*overshoot*)** e oscilações cíclicas sustentadas (efeito pêndulo).
A supressão do termo integral ($K_i = 0$) e a introdução da derivada ($K_d$) conferem amortecimento ótimo, agindo como um freio antecipado à medida que o drone se aproxima do centro.

### 5.3. Sintonização dos Ganhos por Fase de Voo
O projeto utiliza ganhos variáveis dependendo do estágio da missão:
- **Fase 1 (Aproximação Macro, de 18 m até 2 m):**
  - $K_p = 0.4$: garante convergência rápida para a vizinhança da armadilha.
  - $K_d = 0.01$: amortecimento leve de alta velocidade.
  - Velocidade horizontal máxima limitada a: $v_{\max} = 1.5\text{ m/s}$.
- **Fase 2 (Ajuste Fino Micro, de 2 m até o toque):**
  - $K_p = 0.15$: resposta extremamente suave para evitar perturbações no fluxo de ar.
  - $K_d = 0.05$: alto amortecimento derivativo para fixação estática.
  - Velocidade horizontal máxima limitada a: $v_{\max} = 0.02 \text{ a } 0.35\text{ m/s}$.

---

## 6. Mecanismos Especiais de Estabilidade e Segurança

### 6.1. Histerese Ótica de Descida (Manutenção do Campo de Visão - FOV)
Durante a descida, se o drone afundar verticalmente enquanto estiver desalinhado na horizontal, o alvo pode escapar pela borda da imagem.
Para impedir isso, foi implementada uma **Histerese de Dupla Zona**:
- **Zona Segura (40% do centro da imagem):** O drone só é autorizado a descer ($v_z > 0$) se o erro estiver estritamente contido neste retângulo central.
- **Limite de Fuga (60% do centro da imagem):** Se o erro ultrapassar 60%, a descida é imediatamente interrompida ($v_z = 0.0$) e 100% da autoridade de controle é dedicada à correção horizontal.

### 6.2. Frenagem Vertical Estendida
Para dissipar a energia cinética vertical acumulada na descida rápida de 18 m, o sistema aplica um fator de desaceleração linear entre 6 m e 2 m de altitude:

$$f_{\text{frenagem}} = \max\left(0.1, \min\left(1.0, \frac{\text{altitude} - 2.0}{6.0 - 2.0}\right)\right)$$

$$v_z = v_{z,\text{base}} \cdot f_{\text{frenagem}}$$

Isso faz com que o drone alcance a cota de 2 metros suavemente a $0.15\text{ m/s}$, sem qualquer solavanco inercial que pudesse desestabilizar a IMU.

### 6.3. Alinhamento de Guinada (Yaw) via Geometria Isósceles
Para que a garra mecânica capture a alça da gaiola, o drone precisa alinhar seu ângulo de Yaw ($\psi$) com a estrutura física da armadilha.
1. A rede YOLOv11n detecta as 3 alças da tampa da gaiola.
2. O algoritmo calcula as três distâncias euclidianas entre os pares: $d_{01}, d_{12}, d_{20}$.
3. O vértice do triângulo isósceles ($P_{\text{apex}}$) é isolado pela menor diferença absoluta entre os lados adjacentes:
   $$\min\Big(|d_{01} - d_{20}|, |d_{01} - d_{12}|, |d_{12} - d_{20}|\Big)$$
4. O vetor diretor da base ao ápice determina o ângulo atual do alvo:
   $$\theta = \operatorname{atan2}(y_{\text{apex}} - y_{\text{base}}, x_{\text{apex}} - x_{\text{base}})$$
5. O ângulo passa por um **Filtro de Kalman Circular**, que corrige o *wrap-around* periódico no intervalo $[-\pi, \pi]$:
   $$\Delta \theta_{\text{normalizado}} = (\theta_{\text{medido}} - \hat{\theta} + \pi) \pmod{2\pi} - \pi$$
6. O controlador de Yaw atua até manter o erro angular $\le 1.0^\circ$ por 2.0 segundos ininterruptos, travando a rotação em seguida.

### 6.4. Descida Cega Rápida (*Blind Drop* Anti-Deriva no Efeito Solo)
Abaixo de 50 cm de altura, dois fenômenos críticos ocorrem:
1. **Efeito Solo (Ground Effect):** O ar expelido pelas hélices colide com o chão e gera vórtices turbulentos caóticos que tentam empurrar o drone para os lados.
2. **Ponto Cego da Câmera:** O alvo ocupa todo o enquadramento e partes dele saem do campo de visão, degradando as bounding boxes.

**A Solução Algorítmica:**
Após sustentar um alinhamento com precisão milimétrica ($\le 2\text{ mm}$ no alvo com offset) por 2.0 segundos a 0.50 m de altitude:
- A visão computacional é desligada.
- As velocidades laterais são zeradas ($v_x = 0, v_y = 0, \text{yaw\_rate} = 0$).
- É disparado um mergulho vertical rápido e constante ($v_z = 0.35\text{ m/s}$) até o toque físico.
Esse mergulho constante vence as bolhas de recirculação do efeito solo por momento inercial puro, garantindo o toque com desvio médio nulo.

---

## 7. Roteiro Prático: Como Explicar Isso em uma Apresentação ou Banca

Se perguntarem: *"Como o cálculo garante que o drone não perde a estabilidade durante o voo?"*, você pode estruturar sua resposta em 4 pontos objetivos:

1. **"Nós usamos uma arquitetura de Controle em Cascata em duas frequências diferentes:"**
   > *"O ArduPilot cuida da física rápida a 400 Hz (lendo giroscópios e acelerômetros para manter o drone reto no ar e não capotar). O nosso código em Python roda a 20 Hz e só se preocupa em enviar a velocidade que o drone deve ter para chegar ao alvo."*

2. **"Nós não controlamos em pixels, controlamos em metros reais usando o Modelo Pinhole (PBVS):"**
   > *"Controlar por pixels causaria instabilidade porque 100 pixels a 18 metros representam mais de 1,5 metro no chão, mas a 1 metro representam poucos centímetros. Usando a distância focal da câmera e a altitude do drone, invertemos a matriz de projeção Pinhole. Assim, nosso erro é sempre medido em metros físicos reais em qualquer altura."*

3. **"O Filtro de Kalman (EKF) elimina o tremor da IA e compensa atrasos de imagem:"**
   > *"Redes neurais têm jitter e pequenas falhas ocasionais de quadro. O Filtro de Kalman modela a física de velocidade e posição do alvo. Se a câmera tiver um ruído de 5 pixels ou perder o alvo por alguns milissegundos, o filtro ignora o ruído e prevê o movimento matematicamente, mantendo o voo perfeitamente amortecido."*

4. **"O Controlador PD elimina o overshoot porque não usamos o termo Integral:"**
   > *"Como a câmera tem latência de processamento (~50 ms), usar um termo Integral (PID tradicional) faria o drone acumular erro e ficar oscilando como um pêndulo sobre o alvo. O termo Proporcional move o drone e o termo Derivativo calcula a taxa de aproximação e freia suavemente antes do centro. Para a descida final, usamos histerese para não perder o alvo de vista e mergulho cego nos últimos 50 cm para neutralizar a turbulência do efeito solo."*
