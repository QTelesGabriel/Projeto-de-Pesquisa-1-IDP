**CONTEXTO DO PROJETO (TCC EM ENGENHARIA/COMPUTAÇÃO)**

**Visão Geral:**
O projeto consiste no desenvolvimento de um sistema autônomo para um VANT (drone quadricóptero), operando em ambiente simulado, capaz de detectar, alinhar-se e realizar a descida controlada sobre um objeto estático localizado no solo (com um objetivo opcional de captura com garra). O escopo de desenvolvimento é de 4 meses.

**Arquitetura do Sistema (Dividida em 4 Módulos Principais):**

**1. Módulo de Simulação e Ambiente**
*   **Ferramentas:** O ambiente virtual roda no **Gazebo**. O controle de baixo nível da física do drone é feito utilizando o firmware **ArduPilot** em configuração **SITL** (Software-In-The-Loop).
*   **Middleware:** Todo o sistema de comunicação (nós, tópicos, envio de imagens e recebimento de velocidades) é orquestrado pelo **ROS** (Robot Operating System). A comunicação entre o script de controle e o SITL é feita via protocolo **MAVLink/MAVROS**.

**2. Módulo de Percepção Visual (Visão Computacional)**
*   **Câmera:** Uma câmera acoplada ao drone voltada para baixo (nadir).
*   **Abordagem Primária (Clássica):** Segmentação de cor no espaço **HSV** utilizando OpenCV. Aplica-se *thresholding*, binarização e extração de contornos para calcular o **centroide** (coordenadas x, y em pixels) do alvo.
*   **Abordagem Secundária (Robustez via Deep Learning):** Substituição ou complemento do HSV por redes neurais de estágio único focadas em tempo real, como **YOLOv8 nano** ou **YOLOv3 Tiny**. 
*   **Filtragem de Ruído:** Implementação de um **Filtro de Kalman discreto** na saída da percepção para suavizar oscilações (jittering) do centroide/bounding box geradas pelo movimento do drone e prever a posição em caso de falha de detecção.

**3. Módulo de Controle e Tomada de Decisão**
*   **Máquina de Estados Finitos (FSM):** Gerencia os comportamentos: 1) Busca do objeto; 2) Alinhamento horizontal (eixos x, y); 3) Aproximação/descida (eixo z); 4) Pouso/Captura.
*   **Controle em Cascata (Malha Dupla):** 
    *   *Malha Interna (Rápida):* Rodando no piloto automático (ArduPilot SITL), responsável por estabilizar a atitude complexa do drone (Roll, Pitch, Yaw).
    *   *Malha Externa (Lenta/Cinemática):* Um script em Python programado do zero com **Controladores PID discretos** (Proporcional, Integral, Derivativo) atuando nos eixos X, Y e Z.
*   **Visual Servoing:** O erro posicional é calculado baseado na distância do alvo em relação ao centro da imagem. Para mitigar a instabilidade gerada pelo "efeito pêndulo" (quando o drone inclina para transladar e a câmera sai do alvo), o projeto assumirá compensação via matriz de rotação para espaço 3D (PBVS) ou a modelagem de um Gimbal virtual no Gazebo.

**4. Módulo de Integração e Execução**
*   O sistema atua em um loop fechado contínuo: *Câmera Gazebo -> Tópico ROS -> OpenCV/YOLO -> Filtro de Kalman -> Cálculo do Erro -> Controlador PID Python -> Comando MAVLink de Velocidade (Vx, Vy, Vz) -> ArduPilot SITL -> Movimento no Gazebo.*

**Base Bibliográfica e Estado da Arte:**
Este sistema fundamenta-se na literatura recente sobre Controle de VANTs e Deep Learning:
*   *Dinâmica e Simulação:* Validação do modelo Newton-Euler e integração Gazebo/ArduPilot via ROS.
*   *Visão Computacional:* Uso de transformações de 2D (pixels) para 3D (métrico) em inspeções e uso de YOLO para detecção e seguimento de alvos sob oclusões ou ruídos.
*   *Teoria de Controle:* Aplicação de malhas PID em cascata para atitude e posição e as estratégias de *Visual Servoing* projetadas para manter o alvo no campo de visão (FOV) do robô.

**Instrução para a IA:**
A partir deste momento, aja como um especialista em Robótica (ROS/Gazebo), Visão Computacional (OpenCV/YOLO) e Teoria de Controle de VANTs. Considere todas as respostas baseadas nas restrições de tempo (4 meses) e na arquitetura de software/hardware simulado definida acima.
