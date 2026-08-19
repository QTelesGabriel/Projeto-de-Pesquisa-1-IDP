O problema principal não era o PID. Eram sinais de sensores e motores associados aos eixos errados. Assim, quando o ArduPilot tentava corrigir uma inclinação, ele aumentava motores que pioravam o movimento.

Alterações no [model.sdf](/home/gabriel/Projetos/projeto/Projeto-de-Pesquisa-1-IDP/Code/ardupilot_gazebo/models/drone_coaxial/model.sdf):

| Configuração | Antes | Agora | Motivo |
|---|---:|---:|---|
| Massa total | 7,018 kg | 6,700 kg | A massa da base não descontava os oito rotores de 0,05 kg |
| Rotação do IMU | Sem correção | 180° em roll | Alinha acelerômetro e giroscópio ao sistema do ArduPilot |
| Gazebo → NED | Faltavam 90° em yaw | `180 0 90` | Alinha corretamente X/Y entre Gazebo e ArduPilot |
| Rotação máxima | 1200 rad/s | 571,35 rad/s | 571,35 rad/s corresponde aos 5456 rpm do motor real |
| PWM máximo | 1900 | 2000 | Mantém a mesma faixa usada pelo ArduPilot |
| Limite de torque | ±3 Nm | ±1,6 Nm | Próximo do torque máximo real de 1,49 Nm |
| Frequência do IMU | 250 Hz | 1000 Hz | Igual ao modelo oficial e mais adequada ao controlador |

A ausência da rotação do IMU era especialmente grave: a atitude absoluta indicava um sentido, enquanto o giroscópio informava outro. Isso criava realimentação positiva — o controlador aumentava a correção depois que o drone já havia ultrapassado a posição desejada.

Também corrigi a associação dos canais do OctaQuad X:

```text
Canal 0 → frente-direita superior
Canal 1 → frente-esquerda superior
Canal 2 → traseira-esquerda superior
Canal 3 → traseira-direita superior
Canal 4 → frente-esquerda inferior
Canal 5 → frente-direita inferior
Canal 6 → traseira-direita inferior
Canal 7 → traseira-esquerda inferior
```

Antes, os canais 1/2 e 5/6 estavam associados aos braços errados. Portanto, uma correção de roll ou pitch gerava torque no lugar incorreto.

A aerodinâmica dos 16 elementos `LiftDrag` também foi recalibrada para o MN6007 II KV160/12S com P22×6.6:

```text
T = 1,88928×10⁻⁴ · ω²  N
Q = 4,48871×10⁻⁶ · ω²  Nm
```

Foram alterados:

- `area`: `0.005` → `0.0037982` por pá simulada.
- `cda`: `0.10` → `0.56097`.
- Centro aerodinâmico `cp`: `0.084` → `0.18 m`.
- Sentidos CW/CCW mantidos em pares opostos.

Isso fez o modelo produzir empuxo e torque de reação compatíveis. O torque de reação é o que permite ao ArduPilot controlar a guinada; antes ele estava muito abaixo do necessário.

No [drone_coaxial.parm](/home/gabriel/Projetos/projeto/Projeto-de-Pesquisa-1-IDP/Code/ardupilot_gazebo/models/drone_coaxial/drone_coaxial.parm) configurei:

```text
FRAME_CLASS     4      # OctaQuad
FRAME_TYPE      1      # X
MOT_THST_EXPO   1.0    # empuxo proporcional a ω²
MOT_THST_HOVER  0.125
MOT_PWM_MIN     1000
MOT_PWM_MAX     2000
```

O `MOT_THST_EXPO=1.0` é importante porque o plugin converte PWM em velocidade de rotação linearmente, enquanto o empuxo cresce aproximadamente com o quadrado da rotação.

Também atualizei:

- [para_voar.txt](/home/gabriel/Projetos/projeto/Projeto-de-Pesquisa-1-IDP/Code/ardupilot_gazebo/models/drone_coaxial/para_voar.txt) com `-f octa-quad`, arquivo de parâmetros e caminhos corretos.
- [takeoff.py](/home/gabriel/Projetos/projeto/Projeto-de-Pesquisa-1-IDP/Code/drone_app/vision_yolo/takeoff.py:24) para aguardar a altitude desejada e depois entrar em `HOLD`.

Por isso funciona agora: sensores, eixos, matriz dos motores, sentidos de rotação e força dos motores estão coerentes entre si. No teste, o drone estabilizou em 3 m, nivelado, sem deslocamento e com os oito motores igualmente em PWM 1368. Não foi necessário alterar os PIDs principais do ArduPilot.