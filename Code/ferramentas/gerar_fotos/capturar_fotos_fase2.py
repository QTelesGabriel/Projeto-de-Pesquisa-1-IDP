#!/usr/bin/env python3
"""
=============================================================================
Captura Automatizada de Fotos - Fase 2 (Micro Ajuste da Armadilha)
=============================================================================

Este script:
1. Conecta ao drone (DroneKit/MAVLink) e ao tópico da câmera ROS 2 (/camera/image).
2. Se estiver no chão, decola para a altitude máxima da Fase 2 (padrão: 2.0m). Se no ar, estabiliza nela.
3. Aponta o Gimbal em Nadir (-90 graus pitch) com correção Yaw em -90.0 graus (idêntico à missão).
4. Voa em linha reta com precisão até a posição centralizada da armadilha no Gazebo:
   (Gazebo X = 50.0m, Y = 0.0m -> LOCAL_NED: Norte = 0.0m, Leste = 50.0m, Altitude = 2.0m).
5. ESPERA INTERATIVA: Pausa o drone sobre o alvo, transmite o vídeo ao vivo e aguarda
   o usuário pressionar [ENTER] ou [ESPAÇO] para iniciar a captura.
6. VARREDURA VERTICAL COM FOCO EXTREMO (360° + Z):
   - O drone permanece ESTRITAMENTE centralizado no plano horizontal (X e Y fixos, sem desvios).
   - Movimento vertical cíclico entre o Topo (2.00m) e a Base (0.75m).
   - DISTRIBUIÇÃO PONDERADA DE FOTOS:
     * TOPO (2.0m) : 40% das fotos tiradas com giro de 360° (visão completa da gaiola).
     * DESCIDA     : 10% das fotos espaçadas suavemente em altitude (transição).
     * BASE (0.75m): 40% das fotos tiradas com giro de 360° (detalhe máximo das alças).
     * SUBIDA      : 10% das fotos espaçadas suavemente em altitude (transição).
   - Giro contínuo de Yaw durante todo o processo para cobrir todos os ângulos.
7. Teclas ao vivo na tela:
   - [ESPAÇO]: Pausar / Retomar movimento
   - [C]     : Foto manual imediata
   - [J / L] : Gira o gimbal em -90° / +90°
   - [[ / ]] : Ajuste fino do gimbal em -15° / +15°
   - [Q]     : Sair com segurança
=============================================================================
"""

# --- COMPATIBILIDADE DRONEKIT PYTHON 3.10+ ---
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------

import os
import sys
import time
import math
import glob
import re
import select
import argparse
from datetime import datetime
import cv2

# Importações seguras de ROS 2
rclpy = None
Node = None
Image = None
CvBridge = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    pass

# Importações seguras de DroneKit e MAVLink
vehicle_module_ok = True
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
except ImportError:
    vehicle_module_ok = False

# SCRIPT_DIR
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# =============================================================================
# CONSTANTES E CONFIGURAÇÕES PADRÃO
# =============================================================================
DEFAULT_CONEXAO = 'udp:127.0.0.1:14550'

# Posição absoluta da armadilha no mundo Gazebo:
# Gazebo: X = 50.0m (Leste), Y = 0.0m (Norte), Z = 1.0m (Base no solo)
ALVO_NORTE_NED = 0.0   # Gazebo Y (Frente/Norte)
ALVO_LESTE_NED = 50.0  # Gazebo X (Direita/Leste)

# Escopo estrito da Fase 2
ALTURA_MAX_FASE2 = 2.0   # metros (Altitude inicial da Fase 2)
# Em rastreador_fino.py, a câmera atua até 0.75m. A 0.75m o drone trava vel_z=0 e inicia Blind Drop.
ALTURA_MIN_FASE2 = 0.75  # metros (Altitude onde a câmera encerra a visão ativa)

# Rotação padrão do Gimbal em Yaw:
# Configurado em -90 graus para coincidir exatamente com a missão (main.py)
DEFAULT_GIMBAL_YAW = -90.0

DEFAULT_PASTA_SAIDA = os.path.abspath(
    os.path.join(SCRIPT_DIR, "../yolo_label/dataset/imagens")
)


def obter_proximo_numero_captura(pasta_destino):
    """
    Analisa a pasta de imagens e descobre o próximo índice de captura disponível,
    começando em 1 se a pasta estiver vazia.
    """
    os.makedirs(pasta_destino, exist_ok=True)
    arquivos = glob.glob(os.path.join(pasta_destino, "*.jpg")) + glob.glob(os.path.join(pasta_destino, "*.png"))
    maior_num = 0
    padrao = re.compile(r"captura_(\d+)_")

    for f in arquivos:
        nome = os.path.basename(f)
        match = padrao.search(nome)
        if match:
            num = int(match.group(1))
            if num > maior_num:
                maior_num = num

    return maior_num + 1


def aguardar_atributos(vehicle, timeout=30):
    """Espera a telemetria básica do ArduPilot ficar disponível."""
    start = time.time()
    print("[*] Aguardando leitura dos dados de telemetria do drone...")
    while time.time() - start < timeout:
        if vehicle.mode.name and vehicle.location.global_relative_frame and vehicle.is_armable is not None:
            return True
        time.sleep(1)
    return False


def comandar_yaw_absoluto(vehicle, yaw_deg=0.0):
    """
    Comanda o drone a girar e fixar o Yaw absoluto especificado (0 a 360 graus):
    1. Garante WP_YAW_BEHAVIOR = 0 no ArduPilot.
    2. Envia MAV_CMD_CONDITION_YAW para apontar no ângulo desejado.
    """
    yaw_norm = float(yaw_deg) % 360.0
    if yaw_norm < 0:
        yaw_norm += 360.0

    try:
        vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
    except Exception:
        pass

    for sys_id, comp_id in [(1, 1), (0, 0)]:
        try:
            msg_yaw = vehicle.message_factory.command_long_encode(
                sys_id, comp_id,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                yaw_norm,  # param 1: yaw em graus absolutos
                0.0,       # param 2: velocidade padrão
                1,         # param 3: horário
                0,         # param 4: 0 = ângulo absoluto
                0, 0, 0
            )
            vehicle.send_mavlink(msg_yaw)
        except Exception:
            pass

    vehicle.flush()


def travar_proa_norte(vehicle):
    """Atalho para travar a proa em 0 graus (Norte)."""
    comandar_yaw_absoluto(vehicle, 0.0)


def calcular_erro_yaw_deg(veiculo, yaw_alvo_deg):
    """Calcula a diferença angular absoluta (em graus) entre o Yaw real da aeronave e o Yaw desejado."""
    try:
        if veiculo.attitude and veiculo.attitude.yaw is not None:
            yaw_atual_deg = math.degrees(veiculo.attitude.yaw) % 360.0
            yaw_alvo_norm = float(yaw_alvo_deg) % 360.0
            erro = (yaw_alvo_norm - yaw_atual_deg + 180.0) % 360.0 - 180.0
            return abs(erro)
    except Exception:
        pass
    return 0.0




def comandar_posicao_absoluta_local(vehicle, frente_x, direita_y, baixo_z, yaw_rad=0.0):
    """
    Envia comando MAVLink LOCAL_NED com posição X, Y, Z absoluta em relação ao mundo
    e Yaw explicitamente travado em yaw_rad (0.0 rad = Norte).
    """
    ts = getattr(vehicle, '_master', None)
    target_sys = vehicle._master.target_system if ts and vehicle._master.target_system else 1
    target_comp = vehicle._master.target_component if ts and vehicle._master.target_component else 1

    # Typemask para posicionamento X, Y, Z e Yaw absoluto:
    # Ignora: VX (8), VY (16), VZ (32), AX (64), AY (128), AZ (256), YAW_RATE (2048)
    # Usa: X, Y, Z, YAW (Bits 0, 1, 2, 10 em 0; Bit 9 FORCE_SET = 0)
    # 8 + 16 + 32 + 64 + 128 + 256 + 2048 = 2552 = 0b0000100111111000 = 0x09F8
    typemask = 0b0000100111111000

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                        # time_boot_ms
        target_sys, target_comp,  # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        typemask,
        frente_x, direita_y, baixo_z, # Posição Norte, Leste, Down
        0, 0, 0,                  # Velocidades
        0, 0, 0,                  # Acelerações
        yaw_rad, 0                # Yaw desejado (0.0 = Norte)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=0.0):
    """Aponta o Gimbal para baixo e com o ângulo de Yaw especificado."""
    pitch_limitado = max(-135.0, min(45.0, float(pitch_deg)))
    yaw_limitado = max(-160.0, min(160.0, float(yaw_deg)))

    # Flags = 12 (Roll e Pitch estabilizados, Yaw relativo)
    flags = 12
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        0,
        pitch_limitado,
        yaw_limitado,
        math.nan,
        math.nan,
        flags,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def calcular_distancia_horizontal(veiculo, alvo_norte, alvo_leste):
    """Calcula a distância horizontal (metros) até o alvo no referencial local."""
    try:
        loc = veiculo.location.local_frame
        if loc and loc.north is not None and loc.east is not None:
            dn = loc.north - alvo_norte
            dl = loc.east - alvo_leste
            return math.sqrt(dn**2 + dl**2)
    except Exception:
        pass
    return None


BaseNode = Node if Node is not None else object

class ReceptorCameraROS(BaseNode):
    """Nó ROS 2 responsável por receber os frames de vídeo da câmera do Gazebo."""
    def __init__(self):
        if Node is not None:
            super().__init__('receptor_camera_fase2')
        self.bridge = CvBridge() if CvBridge is not None else None
        self.ultimo_frame = None
        self.frame_recebido = False

        if Node is not None:
            self.subscription = self.create_subscription(
                Image,
                '/camera/image',
                self.image_callback,
                10
            )

    def image_callback(self, msg):
        try:
            self.ultimo_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_recebido = True
        except Exception as e:
            pass


def navegar_e_estabilizar(vehicle, alvo_x, alvo_y, alt_desejada, node_camera, tol_h=0.35, tol_alt=0.15, timeout_seg=35.0):
    """
    Navega em malha fechada para as coordenadas absolutas (alvo_x, alvo_y, alt_desejada),
    enviando comandos continuamente e aguardando o drone fisicamente alcançar a altitude e posição!
    """
    print(f"\n[*] Movendo para Posição Absoluta: Norte={alvo_x:.1f}m, Leste={alvo_y:.1f}m, Altura={alt_desejada:.2f}m...")
    inicio = time.time()
    tempo_dentro_tolerancia = None

    while True:
        if rclpy is not None:
            rclpy.spin_once(node_camera, timeout_sec=0.05)

        # Envia comando de posição continuamente (5 Hz) para o ArduPilot não desviar
        comandar_posicao_absoluta_local(vehicle, alvo_x, alvo_y, -alt_desejada, yaw_rad=0.0)

        alt_real = vehicle.location.global_relative_frame.alt
        dist_h = calcular_distancia_horizontal(vehicle, alvo_x, alvo_y)

        erro_alt = abs(alt_real - alt_desejada)
        dist_str = f"{dist_h:.2f}m" if dist_h is not None else "calculando..."
        tempo_dec = time.time() - inicio

        sys.stdout.write(
            f"\r -> Alinhando: Dist H={dist_str} | Alt Real={alt_real:.2f}m (Alvo={alt_desejada:.2f}m, Erro={erro_alt:.2f}m) [{tempo_dec:.1f}s]   "
        )
        sys.stdout.flush()

        # Condição de estabilização simultânea em X, Y e Z
        pos_h_ok = (dist_h is not None and dist_h <= tol_h) or (dist_h is None and tempo_dec >= 15.0)
        pos_alt_ok = (erro_alt <= tol_alt)

        if pos_h_ok and pos_alt_ok:
            if tempo_dentro_tolerancia is None:
                tempo_dentro_tolerancia = time.time()
            elif time.time() - tempo_dentro_tolerancia >= 1.2:
                # Estabilizado por mais de 1.2s dentro da tolerância
                print(f"\n[+] ESTABILIZADO COM SUCESSO! Altura cravada: {alt_real:.2f}m (Erro: {erro_alt*100:.1f}cm).")
                break
        else:
            tempo_dentro_tolerancia = None

        if tempo_dec >= timeout_seg:
            print(f"\n[!] Timeout de alinhamento atingido ({timeout_seg}s). Prosseguindo com altitude atual: {alt_real:.2f}m.")
            break

        time.sleep(0.1)


def aguardar_confirmacao_usuario(vehicle, node_camera, nome_janela, yaw_gimbal_inicial, altura=2.0):
    """
    Mantém o drone pairado na altitude fixa sobre a armadilha, transmite a câmera ao vivo,
    permite ajustes de rotação do Gimbal e aguarda o usuário pressionar ENTER ou ESPAÇO
    antes de iniciar a rotação contínua e as fotos.
    """
    print("\n" + "=" * 75)
    print(" [AGUARDANDO SUA CONFIRMAÇÃO PARA INICIAR AS FOTOS]")
    print(f" Drone pairando estabilizado a {altura:.2f}m centralizado sobre a armadilha.")
    print(" A câmera está transmitindo ao vivo na janela gráfica.")
    print("")
    print(" --> Pressione [ENTER] ou [ESPAÇO] para começar o giro de fotos em 360°!")
    print(" --> [J] / [L]     : Gira a câmera em -90° / +90°")
    print(" --> [ [ ] / [ ] ] : Ajuste fino da câmera em -15° / +15°")
    print(" --> [C]           : Dispara uma foto manual de teste")
    print(" --> [Q / ESC]     : Cancelar e sair")
    print("=" * 75 + "\n")

    yaw_gimbal = yaw_gimbal_inicial
    foto_teste_num = 1

    while True:
        if rclpy is not None:
            rclpy.spin_once(node_camera, timeout_sec=0.04)

        # Continua mantendo a posição centralizada na altitude configurada
        comandar_posicao_absoluta_local(vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, -altura, yaw_rad=0.0)

        # Monta exibição com HUD de instrução
        if node_camera.ultimo_frame is not None:
            frame_raw = node_camera.ultimo_frame.copy()
            frame_display = cv2.resize(frame_raw, (960, 540))

            alt_real = vehicle.location.global_relative_frame.alt
            hud_titulo = "[AGUARDANDO CONFIRMACAO] APERTE ENTER OU ESPACO PARA INICIAR"
            hud_status = f"Alt: {alt_real:.2f}m | Proa: 0.0 (Norte) | Gimbal Yaw: {yaw_gimbal:+.1f} deg"
            hud_comandos = "ENTER/ESPACO: Iniciar | J/L: Girar 90 | [/]: Fino 15 | C: Foto Teste | Q: Sair"

            cv2.rectangle(frame_display, (0, 0), (960, 70), (0, 45, 120), -1)
            cv2.putText(frame_display, hud_titulo, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(frame_display, hud_status, (15, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)
            cv2.putText(frame_display, hud_comandos, (15, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
            
            # Mira central
            cv2.drawMarker(frame_display, (480, 270), (0, 255, 255), cv2.MARKER_CROSS, 20, 1)

            cv2.imshow(nome_janela, frame_display)
        else:
            # Quadro de espera se a imagem ainda estiver conectando
            import numpy as np
            quadro_espera = np.zeros((540, 960, 3), dtype=np.uint8)
            cv2.putText(quadro_espera, "Aguardando frames do topico /camera/image...", (120, 270),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow(nome_janela, quadro_espera)

        # Leitura da tecla na janela OpenCV
        tecla = cv2.waitKey(20) & 0xFF

        # Leitura não-bloqueante de ENTER no terminal
        enter_no_terminal = False
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                enter_no_terminal = True
        except Exception:
            pass

        if tecla in [13, 32] or enter_no_terminal:  # ENTER (13), ESPAÇO (32) ou ENTER no terminal
            print("\n[+] CONFIRMAÇÃO RECEBIDA COM SUCESSO! Iniciando captura de fotos agora...\n")
            time.sleep(0.5)
            return yaw_gimbal

        elif tecla == ord('c') or tecla == ord('C'):
            if node_camera.ultimo_frame is not None:
                caminho_teste = os.path.join(DEFAULT_PASTA_SAIDA, f"teste_previa_{foto_teste_num}.jpg")
                cv2.imwrite(caminho_teste, node_camera.ultimo_frame)
                print(f"[TESTE] Foto de teste salva em: {caminho_teste}")
                foto_teste_num += 1

        elif tecla == ord('j') or tecla == ord('J'):
            yaw_gimbal = yaw_gimbal - 90.0
            if yaw_gimbal < -160.0: yaw_gimbal += 360.0
            print(f"[GIMBAL] Girando para a esquerda (-90°): {yaw_gimbal:+.1f}°")
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla == ord('l') or tecla == ord('L'):
            yaw_gimbal = yaw_gimbal + 90.0
            if yaw_gimbal > 160.0: yaw_gimbal -= 360.0
            print(f"[GIMBAL] Girando para a direita (+90°): {yaw_gimbal:+.1f}°")
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla == ord('['):
            yaw_gimbal = max(-160.0, yaw_gimbal - 15.0)
            print(f"[GIMBAL] Ajuste fino (-15°): {yaw_gimbal:+.1f}°")
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla == ord(']'):
            yaw_gimbal = min(160.0, yaw_gimbal + 15.0)
            print(f"[GIMBAL] Ajuste fino (+15°): {yaw_gimbal:+.1f}°")
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla in [ord('q'), ord('Q'), 27]:
            print("\n[!] Captura cancelada pelo usuário.")
            raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description="Coletor automatizado de fotos para Fase 2 - Varredura Vertical com Foco nos Extremos.")
    parser.add_argument("--conexao", default=DEFAULT_CONEXAO, help="Porta MAVLink de conexão com o drone")
    parser.add_argument("--fotos", type=int, default=500, help="Quantidade total de fotos a serem capturadas (padrão: 500)")
    parser.add_argument("--alt-max", type=float, default=ALTURA_MAX_FASE2, help=f"Altitude máxima / topo da Fase 2 (padrão: {ALTURA_MAX_FASE2}m)")
    parser.add_argument("--alt-min", type=float, default=ALTURA_MIN_FASE2, help=f"Altitude mínima / base da Fase 2 (padrão: {ALTURA_MIN_FASE2}m)")
    parser.add_argument("--ciclos", type=int, default=2, help="Número de ciclos de subida e descida (padrão: 2)")
    parser.add_argument("--pct-extremos", type=float, default=40.0, help="Porcentagem de fotos em cada extremo (padrão: 40%% no topo, 40%% na base, 20%% no trajeto)")
    parser.add_argument("--velocidade-yaw", type=float, default=6.0, help="Velocidade de rotação em graus/segundo (padrão: 6.0 deg/s)")
    parser.add_argument("--velocidade-z", type=float, default=0.12, help="Velocidade vertical de subida/descida em m/s (padrão: 0.12 m/s)")
    parser.add_argument("--gimbal-yaw", type=float, default=DEFAULT_GIMBAL_YAW, help=f"Ângulo de Yaw do Gimbal em graus (padrão: {DEFAULT_GIMBAL_YAW}°, idêntico à missão)")
    parser.add_argument("--saida", default=DEFAULT_PASTA_SAIDA, help="Diretório onde as fotos serão salvas")
    parser.add_argument("--sem-auto", action="store_true", help="Desativa foto automática (apenas manual na tecla C)")
    args = parser.parse_args()

    pasta_saida = os.path.abspath(args.saida)
    os.makedirs(pasta_saida, exist_ok=True)
    contador_foto = obter_proximo_numero_captura(pasta_saida)

    print("=" * 75)
    print(" COLETOR DE FOTOS PARA FASE 2 - VARREDURA VERTICAL COM FOCO EXTREMO")
    print("=" * 75)
    print(f" Posição Centralizada   : Norte={ALVO_NORTE_NED:.1f}m, Leste={ALVO_LESTE_NED:.1f}m (Gazebo X=50.0m, Y=0.0m)")
    print(f" Faixa de Altitude      : {args.alt_min:.2f}m (Base) até {args.alt_max:.2f}m (Topo)")
    print(f" Distribuição de Fotos  : {args.pct_extremos:.0f}% no Topo, {args.pct_extremos:.0f}% na Base, {100.0 - 2*args.pct_extremos:.0f}% na Transição")
    print(f" Ciclos de Varredura    : {args.ciclos} ciclo(s) completos (Desce e Sobe)")
    print(f" Velocidades            : Z = {args.velocidade_z:.2f} m/s | Yaw = {args.velocidade_yaw:.1f}°/s")
    print(f" Yaw do Gimbal          : {args.gimbal_yaw:+.1f}° (Configurado idêntico à missão)")
    print(f" Meta de Fotos          : {args.fotos} imagens")
    print(f" Próxima Foto Começa em : captura_{contador_foto:03d}_...")
    print(f" Pasta de Destino       : {pasta_saida}")
    print("=" * 75)

    # Verificação de dependências em tempo de execução
    if rclpy is None:
        print("\n[ERRO] ROS 2 (rclpy) não foi encontrado neste terminal!")
        print("Antes de rodar, ative o ambiente ROS 2 com:")
        print("  source /opt/ros/jazzy/setup.bash\n")
        sys.exit(1)

    if not vehicle_module_ok:
        print("\n[ERRO] DroneKit ou pymavlink não estão instalados!")
        sys.exit(1)

    # Importa módulos de missão do projeto
    MISSAO_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../missao"))
    if MISSAO_DIR not in sys.path:
        sys.path.append(MISSAO_DIR)

    from controle_voo.takeoff import arm_and_takeoff
    from controle_voo.movimento import configurar_parametro, enviar_velocidade

    # 1. Inicializa ROS 2
    rclpy.init()
    node_camera = ReceptorCameraROS()

    # 2. Conecta ao Drone
    print(f"\n[*] Conectando ao drone via MAVLink em {args.conexao}...")
    try:
        vehicle = connect(args.conexao, wait_ready=False, heartbeat_timeout=15)
        if not aguardar_atributos(vehicle):
            print("[!] ERRO: Telemetria do drone não respondeu. Feche o QGroundControl se aberto.")
            sys.exit(1)
    except Exception as e:
        print(f"[!] ERRO CRÍTICO AO CONECTAR: {e}")
        sys.exit(1)

    print("[+] Drone conectado com sucesso!")

    # Trava Yaw no ArduPilot e estabiliza Gimbal
    print("[+] Travando proa do drone estritamente no Norte (0.0° - WP_YAW_BEHAVIOR = 0)...")
    travar_proa_norte(vehicle)
    yaw_gimbal_atual = args.gimbal_yaw
    print(f"[+] Apontando Gimbal para Nadir (-90°) com Yaw={yaw_gimbal_atual:+.1f}°...")
    ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal_atual)

    # 3. Decolagem (se no chão) ou estabilização no Topo (alt_max)
    alt_inicial = vehicle.location.global_relative_frame.alt
    if alt_inicial < 0.5:
        print(f"[!] Drone no solo ({alt_inicial:.2f}m). Decolando para {args.alt_max:.1f}m...")
        arm_and_takeoff(vehicle, args.alt_max)
    else:
        print(f"[+] Drone já em voo ({alt_inicial:.2f}m).")

    # Garante a proa travada e estabilização na altitude antes de transladar
    travar_proa_norte(vehicle)
    navegar_e_estabilizar(
        vehicle,
        alvo_x=0.0,
        alvo_y=0.0,
        alt_desejada=args.alt_max,
        node_camera=node_camera,
        tol_h=1.0,
        tol_alt=0.15,
        timeout_seg=15.0
    )

    # 4. Voa em linha reta até o centro exato da armadilha no Topo
    print(f"\n[+] Deslocando para o centro da armadilha (Norte={ALVO_NORTE_NED:.1f}m, Leste={ALVO_LESTE_NED:.1f}m, Altura={args.alt_max:.2f}m)...")
    travar_proa_norte(vehicle)
    navegar_e_estabilizar(
        vehicle,
        alvo_x=ALVO_NORTE_NED,
        alvo_y=ALVO_LESTE_NED,
        alt_desejada=args.alt_max,
        node_camera=node_camera,
        tol_h=0.25,
        tol_alt=0.10,
        timeout_seg=35.0
    )
    travar_proa_norte(vehicle)

    # 5. Inicia janela OpenCV
    nome_janela = "Camera Gazebo - Coletor Fase 2"
    cv2.namedWindow(nome_janela, cv2.WINDOW_AUTOSIZE)

    # 6. ESPERA INTERATIVA: Pausa o drone sobre o alvo e aguarda confirmação do usuário
    yaw_gimbal_atual = aguardar_confirmacao_usuario(vehicle, node_camera, nome_janela, yaw_gimbal_atual, altura=args.alt_max)

    print("\n" + "=" * 75)
    print(" INICIANDO CAPTURA COM VARREDURA VERTICAL E GIRO DE YAW")
    print("=" * 75)
    print(" Atalhos na Janela de Vídeo:")
    print("  [ESPAÇO]      : Pausar / Retomar Movimento")
    print("  [C]           : Tirar Foto Manual")
    print("  [J] / [L]     : Girar Gimbal em -90° / +90°")
    print("  [ [ ] / [ ] ] : Ajuste Fino do Gimbal (-15° / +15°)")
    print("  [Q / ESC]     : Sair com Segurança")
    print("=" * 75)

    # Cálculo da cota de fotos por ciclo e por estágio
    fotos_por_ciclo = args.fotos // max(1, args.ciclos)
    meta_topo = int(round(fotos_por_ciclo * (args.pct_extremos / 100.0)))
    meta_base = int(round(fotos_por_ciclo * (args.pct_extremos / 100.0)))
    sobra = max(0, fotos_por_ciclo - meta_topo - meta_base)
    meta_descida = max(1, sobra // 2)
    meta_subida = max(1, sobra - meta_descida)

    # Passos de acionamento
    graus_giro_estagio = 360.0 / float(args.ciclos)
    passo_yaw_topo = graus_giro_estagio / float(max(1, meta_topo))
    passo_yaw_base = graus_giro_estagio / float(max(1, meta_base))

    dist_z = abs(args.alt_max - args.alt_min)
    passo_alt_descida = dist_z / float(max(1, meta_descida))
    passo_alt_subida = dist_z / float(max(1, meta_subida))

    fotos_salvas = 0
    em_pausa = False
    graus_girados = 0.0

    pos_norte = ALVO_NORTE_NED
    pos_leste = ALVO_LESTE_NED

    ciclo = 1
    estado = 'TOPO'
    alt_comando = args.alt_max
    fotos_estagio = 0

    ref_yaw_foto = 0.0
    ref_alt_foto = alt_comando

    try:
        yaw_base_deg = (math.degrees(vehicle.attitude.yaw) % 360.0) if (vehicle.attitude and vehicle.attitude.yaw is not None) else 0.0
    except Exception:
        yaw_base_deg = 0.0

    t_anterior = time.time()

    print(f"\n[CICLO 1/{args.ciclos}] Iniciando no TOPO ({args.alt_max:.2f}m) - Meta: {meta_topo} fotos concentradas...")

    try:
        while fotos_salvas < args.fotos:
            t_agora = time.time()
            dt = t_agora - t_anterior
            t_anterior = t_agora

            if rclpy is not None:
                rclpy.spin_once(node_camera, timeout_sec=0.01)

            if not em_pausa:
                graus_girados += args.velocidade_yaw * dt

                # Evolução da altitude do comando conforme o estado atual
                if estado == 'TOPO':
                    alt_comando = args.alt_max
                elif estado == 'DESCENDO':
                    alt_comando = max(args.alt_min, alt_comando - args.velocidade_z * dt)
                elif estado == 'BASE':
                    alt_comando = args.alt_min
                elif estado == 'SUBINDO':
                    alt_comando = min(args.alt_max, alt_comando + args.velocidade_z * dt)

            yaw_desejado = (yaw_base_deg + graus_girados) % 360.0
            yaw_rad = math.radians(yaw_desejado)

            # Trava o drone rigorosamente no centro (pos_norte, pos_leste), com a altitude comandada e Yaw
            comandar_posicao_absoluta_local(vehicle, pos_norte, pos_leste, -alt_comando, yaw_rad=yaw_rad)

            # Telemetria
            try:
                alt_real = vehicle.location.global_relative_frame.alt
            except Exception:
                alt_real = alt_comando

            try:
                yaw_real_deg = (math.degrees(vehicle.attitude.yaw) % 360.0) if (vehicle.attitude and vehicle.attitude.yaw is not None) else yaw_desejado
            except Exception:
                yaw_real_deg = yaw_desejado

            # Renderiza HUD gráfico informativo
            if node_camera.ultimo_frame is not None:
                frame_raw = node_camera.ultimo_frame.copy()
                frame_display = cv2.resize(frame_raw, (960, 540))

                pct_progresso = (fotos_salvas / float(args.fotos)) * 100.0
                hud_modo = "[PAUSADO]" if em_pausa else f"[{estado} - CICLO {ciclo}/{args.ciclos}]"
                hud_linha1 = f"{hud_modo} Fotos: {fotos_salvas}/{args.fotos} ({pct_progresso:.1f}%) | Estagio: {fotos_estagio}"
                hud_linha2 = f"Alt Real: {alt_real:.2f}m (Cmd: {alt_comando:.2f}m) | Faixa: [{args.alt_min:.2f}m - {args.alt_max:.2f}m]"
                hud_linha3 = f"Yaw: {yaw_real_deg:.1f} deg | Gimbal: {yaw_gimbal_atual:+.0f} deg | [ESPACO]: Pausa | [C]: Foto"

                cv2.rectangle(frame_display, (0, 0), (960, 68), (20, 20, 20), -1)
                cor_status = (0, 165, 255) if em_pausa else (0, 255, 0)
                cv2.putText(frame_display, hud_linha1, (15, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, cor_status, 2)
                cv2.putText(frame_display, hud_linha2, (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
                cv2.putText(frame_display, hud_linha3, (15, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                cv2.drawMarker(frame_display, (480, 270), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)

                cv2.imshow(nome_janela, frame_display)

            tecla = cv2.waitKey(15) & 0xFF

            tirar_foto_manual = False
            if tecla == ord('c') or tecla == ord('C'):
                tirar_foto_manual = True

            elif tecla == 32:  # ESPAÇO
                em_pausa = not em_pausa
                t_anterior = time.time()
                print(f"\n[!] {'PAUSA ATIVADA' if em_pausa else 'MOVIMENTO RETOMADO'}")

            elif tecla == ord('j') or tecla == ord('J'):
                yaw_gimbal_atual = (yaw_gimbal_atual - 90.0)
                if yaw_gimbal_atual < -160.0: yaw_gimbal_atual += 360.0
                print(f"\n[GIMBAL] Girando para a esquerda (-90°): {yaw_gimbal_atual:+.1f}°")
                ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal_atual)

            elif tecla == ord('l') or tecla == ord('L'):
                yaw_gimbal_atual = (yaw_gimbal_atual + 90.0)
                if yaw_gimbal_atual > 160.0: yaw_gimbal_atual -= 360.0
                print(f"\n[GIMBAL] Girando para a direita (+90°): {yaw_gimbal_atual:+.1f}°")
                ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal_atual)

            elif tecla == ord('['):
                yaw_gimbal_atual = max(-160.0, yaw_gimbal_atual - 15.0)
                print(f"\n[GIMBAL] Ajuste fino (-15°): {yaw_gimbal_atual:+.1f}°")
                ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal_atual)

            elif tecla == ord(']'):
                yaw_gimbal_atual = min(160.0, yaw_gimbal_atual + 15.0)
                print(f"\n[GIMBAL] Ajuste fino (+15°): {yaw_gimbal_atual:+.1f}°")
                ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal_atual)

            elif tecla in [ord('q'), ord('Q'), 27]:
                print("\n[!] Encerrando por solicitação do usuário.")
                raise KeyboardInterrupt

            # Disparo de foto automática baseado no estado
            disparar_foto_auto = False
            if not args.sem_auto and not em_pausa and node_camera.ultimo_frame is not None:
                if estado == 'TOPO':
                    if (graus_girados - ref_yaw_foto) >= passo_yaw_topo:
                        ref_yaw_foto += passo_yaw_topo
                        disparar_foto_auto = True
                elif estado == 'DESCENDO':
                    if (ref_alt_foto - alt_comando) >= passo_alt_descida:
                        ref_alt_foto -= passo_alt_descida
                        disparar_foto_auto = True
                elif estado == 'BASE':
                    if (graus_girados - ref_yaw_foto) >= passo_yaw_base:
                        ref_yaw_foto += passo_yaw_base
                        disparar_foto_auto = True
                elif estado == 'SUBINDO':
                    if (alt_comando - ref_alt_foto) >= passo_alt_subida:
                        ref_alt_foto += passo_alt_subida
                        disparar_foto_auto = True

            # Salva a foto se disparada
            if (disparar_foto_auto or tirar_foto_manual) and node_camera.ultimo_frame is not None:
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
                nome_arq = f"captura_{contador_foto:03d}_{timestamp_str}.jpg"
                caminho_foto = os.path.join(pasta_saida, nome_arq)

                cv2.imwrite(caminho_foto, node_camera.ultimo_frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
                fotos_salvas += 1
                contador_foto += 1
                fotos_estagio += 1

                pct = (fotos_salvas / float(args.fotos)) * 100.0
                tag = "[MANUAL]" if tirar_foto_manual else f"[{estado}]"
                print(f"{tag:10s} Foto {fotos_salvas:3d}/{args.fotos} ({pct:5.1f}%) | Alt: {alt_real:.2f}m (Cmd: {alt_comando:.2f}m) | Yaw: {yaw_real_deg:5.1f}° | Ciclo: {ciclo}/{args.ciclos}")

                if fotos_salvas >= args.fotos:
                    break

            # Transições da Máquina de Estados
            if estado == 'TOPO' and fotos_estagio >= meta_topo:
                estado = 'DESCENDO'
                fotos_estagio = 0
                ref_alt_foto = alt_comando
                print(f"\n[+] TOPO CONCLUÍDO ({meta_topo} fotos). Iniciando DESCIDA suave ({args.alt_max:.2f}m -> {args.alt_min:.2f}m)...\n")

            elif estado == 'DESCENDO' and (fotos_estagio >= meta_descida or alt_comando <= args.alt_min + 0.01):
                alt_comando = args.alt_min
                estado = 'BASE'
                fotos_estagio = 0
                ref_yaw_foto = graus_girados
                print(f"\n[+] BASE ALCANÇADA ({args.alt_min:.2f}m). Iniciando captura concentrada na BASE ({meta_base} fotos)...\n")

            elif estado == 'BASE' and fotos_estagio >= meta_base:
                estado = 'SUBINDO'
                fotos_estagio = 0
                ref_alt_foto = alt_comando
                print(f"\n[+] BASE CONCLUÍDA ({meta_base} fotos). Iniciando SUBIDA suave ({args.alt_min:.2f}m -> {args.alt_max:.2f}m)...\n")

            elif estado == 'SUBINDO' and (fotos_estagio >= meta_subida or alt_comando >= args.alt_max - 0.01):
                alt_comando = args.alt_max
                ciclo += 1
                if ciclo <= args.ciclos and fotos_salvas < args.fotos:
                    estado = 'TOPO'
                    fotos_estagio = 0
                    ref_yaw_foto = graus_girados
                    print(f"\n[+] CICLO {ciclo-1} CONCLUÍDO! Iniciando CICLO {ciclo}/{args.ciclos} no TOPO ({args.alt_max:.2f}m)...\n")
                else:
                    estado = 'TOPO'
                    ref_yaw_foto = graus_girados

    except KeyboardInterrupt:
        print("\n[!] Captura interrompida pelo usuário.")

    finally:
        print("\n" + "=" * 75)
        print(" FINALIZANDO MISSÃO DE CAPTURA")
        print("=" * 75)
        print(f" Total de Fotos Capturadas : {fotos_salvas}")
        print(f" Pasta de Salvamento       : {pasta_saida}")
        print(f"[*] Subindo drone com segurança para {args.alt_max:.2f}m...")
        try:
            comandar_posicao_absoluta_local(vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, -args.alt_max, yaw_rad=0.0)
            enviar_velocidade(vehicle, 0.0, 0.0, 0.0)
        except Exception:
            pass

        cv2.destroyAllWindows()
        node_camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        vehicle.close()
        print("[+] Concluído com sucesso!")


if __name__ == '__main__':
    main()
