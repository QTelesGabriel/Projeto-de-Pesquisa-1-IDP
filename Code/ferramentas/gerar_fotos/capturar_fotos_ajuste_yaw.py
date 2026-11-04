#!/usr/bin/env python3
"""
=============================================================================
Captura Automatizada de Fotos - Foco na Altura de Ajuste de Yaw (2.00m a 1.50m)
SEM ROTAÇÃO NO YAW (Variação Pura de Altitude)
=============================================================================

Este script captura 100 fotos com proa estritamente fixa (0.0° / Norte),
variando apenas a altitude entre 2.00m e a altura de ajuste de Yaw (1.50m):
- 30 fotos na descida superior de aproximação (2.00m até 1.60m);
- 70 fotos concentradas na zona de ajuste do ângulo (em torno de 1.50m):
  * 35 fotos na descida lenta cruzando 1.50m (1.60m -> 1.48m);
  * 35 fotos na subida lenta pela zona de ajuste (1.48m -> 1.55m).

A rotação da armadilha será coberta via Data Augmentation no treinamento
do YOLO (parâmetro 'degrees=180.0').

Uso:
  source /opt/ros/jazzy/setup.bash
  python3 capturar_fotos_ajuste_yaw.py
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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# =============================================================================
# CONSTANTES E CONFIGURAÇÕES PADRÃO
# =============================================================================
DEFAULT_CONEXAO = 'udp:127.0.0.1:14550'

# Posição absoluta da armadilha no mundo Gazebo (NED)
ALVO_NORTE_NED = 0.0   # Gazebo Y (Norte)
ALVO_LESTE_NED = 50.0  # Gazebo X (Leste)

# Altitudes da missão (Fase 1 -> Fase 2)
ALT_TOPO = 2.00        # Topo da Fase 2 (2.0 metros)
ALT_YAW_ALVO = 1.50    # Altura exata em que o drone executa o ajuste de Yaw na Fase 2
ALT_YAW_MIN = 1.48     # Limite inferior da zona de micro-variação em torno de 1.50m
ALT_YAW_MAX = 1.55     # Limite superior da zona de micro-variação em torno de 1.50m
ALT_DIVISAO = 1.60     # Ponto de transição entre descida rápida e zona crítica

# Total de 100 fotos com concentração na altura de 1.50m
DEFAULT_FOTOS_TOTAL = 100
DEFAULT_FOTOS_ALTA = 30    # Faixa superior [2.00m -> 1.60m]
DEFAULT_FOTOS_YAW_1 = 35   # Zona crítica descendo [1.60m -> 1.48m]
DEFAULT_FOTOS_YAW_2 = 35   # Zona crítica subindo [1.48m -> 1.55m]

DEFAULT_VELOCIDADE_Z_ALTA = 0.05     # m/s (Descida suave na faixa alta)
DEFAULT_VELOCIDADE_Z_FINA = 0.025    # m/s (Descida/subida ultra-lenta na zona de 1.50m)
DEFAULT_VELOCIDADE_RETORNO = 0.20    # m/s (Retorno à altitude de segurança)
DEFAULT_GIMBAL_YAW = -90.0           # Nadir com -90° (idêntico à missão)

DEFAULT_PASTA_SAIDA = os.path.abspath(
    os.path.join(SCRIPT_DIR, "../yolo_label/dataset/imagens")
)


def obter_proximo_numero_captura(pasta_destino):
    """Descobre o próximo índice de captura disponível na pasta."""
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


def comandar_posicao_absoluta_local(vehicle, frente_x, direita_y, baixo_z, yaw_rad=0.0):
    """
    Envia comando MAVLink LOCAL_NED com posição X, Y, Z absoluta e Yaw fixado (0.0 rad = Norte).
    """
    ts = getattr(vehicle, '_master', None)
    target_sys = vehicle._master.target_system if ts and vehicle._master.target_system else 1
    target_comp = vehicle._master.target_component if ts and vehicle._master.target_component else 1

    typemask = 0b0000100111111000

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        target_sys, target_comp,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        typemask,
        frente_x, direita_y, baixo_z,
        0, 0, 0,
        0, 0, 0,
        yaw_rad, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=0.0):
    """Aponta o Gimbal para baixo e com o ângulo de Yaw especificado."""
    pitch_limitado = max(-135.0, min(45.0, float(pitch_deg)))
    yaw_limitado = max(-160.0, min(160.0, float(yaw_deg)))

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
    """Calcula a distância horizontal (metros) até o alvo."""
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
    """Nó ROS 2 responsável por receber os frames da câmera do Gazebo."""
    def __init__(self):
        if Node is not None:
            super().__init__('receptor_camera_fotos_yaw')
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
        except Exception:
            pass


def navegar_e_estabilizar(vehicle, alvo_x, alvo_y, alt_desejada, node_camera, tol_h=0.35, tol_alt=0.15, timeout_seg=35.0):
    """Navega para posição absoluta em malha fechada mantendo Yaw fixo no Norte."""
    print(f"\n[*] Movendo para Posição: Norte={alvo_x:.1f}m, Leste={alvo_y:.1f}m, Alt={alt_desejada:.2f}m...")
    inicio = time.time()
    tempo_dentro_tolerancia = None

    while True:
        if rclpy is not None:
            rclpy.spin_once(node_camera, timeout_sec=0.05)

        comandar_posicao_absoluta_local(vehicle, alvo_x, alvo_y, -alt_desejada, yaw_rad=0.0)

        alt_real = vehicle.location.global_relative_frame.alt
        dist_h = calcular_distancia_horizontal(vehicle, alvo_x, alvo_y)
        erro_alt = abs(alt_real - alt_desejada)
        dist_str = f"{dist_h:.2f}m" if dist_h is not None else "calculando..."
        tempo_dec = time.time() - inicio

        sys.stdout.write(
            f"\r -> Alinhando: Dist H={dist_str} | Alt Real={alt_real:.2f}m (Alvo={alt_desejada:.2f}m) [{tempo_dec:.1f}s]   "
        )
        sys.stdout.flush()

        pos_h_ok = (dist_h is not None and dist_h <= tol_h) or (dist_h is None and tempo_dec >= 15.0)
        pos_alt_ok = (erro_alt <= tol_alt)

        if pos_h_ok and pos_alt_ok:
            if tempo_dentro_tolerancia is None:
                tempo_dentro_tolerancia = time.time()
            elif time.time() - tempo_dentro_tolerancia >= 1.2:
                print(f"\n[+] ESTABILIZADO! Altura: {alt_real:.2f}m (Proa 0.0°).")
                break
        else:
            tempo_dentro_tolerancia = None

        if tempo_dec >= timeout_seg:
            print(f"\n[!] Timeout de estabilização ({timeout_seg}s). Continuando com altura atual: {alt_real:.2f}m.")
            break

        time.sleep(0.1)


def aguardar_confirmacao_usuario(vehicle, node_camera, nome_janela, yaw_gimbal_inicial, altura=2.0):
    """Paira sobre o alvo, transmite feed de vídeo ao vivo e aguarda confirmação do usuário."""
    print("\n" + "=" * 75)
    print(" [AGUARDANDO CONFIRMAÇÃO PARA INICIAR A CAPTURA]")
    print(f" Drone estabilizado a {altura:.2f}m sobre a armadilha.")
    print(" Proa: RIGOROSAMENTE FIXA no Norte (0.0°) - SEM GIRO DE YAW.")
    print(" Câmera transmitindo ao vivo na janela gráfica.")
    print("")
    print(" --> Pressione [ENTER] ou [ESPAÇO] para iniciar a captura de 100 fotos!")
    print("     * 30 fotos na descida superior (2.00m -> 1.60m)")
    print("     * 70 fotos concentradas em torno de 1.50m (35 descendo + 35 subindo)")
    print(" --> [J] / [L]     : Gira o Gimbal em -90° / +90°")
    print(" --> [ [ ] / [ ] ] : Ajuste fino do Gimbal em -15° / +15°")
    print(" --> [C]           : Dispara uma foto manual de teste")
    print(" --> [Q / ESC]     : Cancelar e sair")
    print("=" * 75 + "\n")

    yaw_gimbal = yaw_gimbal_inicial
    foto_teste_num = 1

    while True:
        if rclpy is not None:
            rclpy.spin_once(node_camera, timeout_sec=0.04)

        comandar_posicao_absoluta_local(vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, -altura, yaw_rad=0.0)

        if node_camera.ultimo_frame is not None:
            frame_raw = node_camera.ultimo_frame.copy()
            frame_display = cv2.resize(frame_raw, (960, 540))

            alt_real = vehicle.location.global_relative_frame.alt
            hud_titulo = "[AGUARDANDO CONFIRMACAO] PRESSIONE ENTER OU ESPACO"
            hud_status = f"Alt: {alt_real:.2f}m | Proa: 0.0 deg (FIXA - SEM GIRO) | Gimbal: {yaw_gimbal:+.0f} deg"
            hud_comandos = "ENTER/ESPACO: Iniciar | J/L: Girar Gimbal | C: Foto Teste | Q: Sair"

            cv2.rectangle(frame_display, (0, 0), (960, 70), (0, 45, 120), -1)
            cv2.putText(frame_display, hud_titulo, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(frame_display, hud_status, (15, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)
            cv2.putText(frame_display, hud_comandos, (15, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
            cv2.drawMarker(frame_display, (480, 270), (0, 255, 255), cv2.MARKER_CROSS, 20, 1)

            cv2.imshow(nome_janela, frame_display)
        else:
            import numpy as np
            quadro = np.zeros((540, 960, 3), dtype=np.uint8)
            cv2.putText(quadro, "Conectando ao topico /camera/image...", (200, 270),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(nome_janela, quadro)

        tecla = cv2.waitKey(20) & 0xFF

        enter_terminal = False
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                enter_terminal = True
        except Exception:
            pass

        if tecla in [13, 32] or enter_terminal:
            print("\n[+] Confirmação recebida! Iniciando captura de fotos agora...\n")
            time.sleep(0.5)
            return yaw_gimbal

        elif tecla in [ord('c'), ord('C')]:
            if node_camera.ultimo_frame is not None:
                caminho_teste = os.path.join(DEFAULT_PASTA_SAIDA, f"teste_previa_{foto_teste_num}.jpg")
                cv2.imwrite(caminho_teste, node_camera.ultimo_frame)
                print(f"[TESTE] Foto de teste salva: {caminho_teste}")
                foto_teste_num += 1

        elif tecla in [ord('j'), ord('J')]:
            yaw_gimbal = yaw_gimbal - 90.0
            if yaw_gimbal < -160.0: yaw_gimbal += 360.0
            print(f"[GIMBAL] Girando: {yaw_gimbal:+.1f}°")
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla in [ord('l'), ord('L')]:
            yaw_gimbal = yaw_gimbal + 90.0
            if yaw_gimbal > 160.0: yaw_gimbal -= 360.0
            print(f"[GIMBAL] Girando: {yaw_gimbal:+.1f}°")
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla == ord('['):
            yaw_gimbal = max(-160.0, yaw_gimbal - 15.0)
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla == ord(']'):
            yaw_gimbal = min(160.0, yaw_gimbal + 15.0)
            ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal)

        elif tecla in [ord('q'), ord('Q'), 27]:
            print("\n[!] Captura cancelada.")
            raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description="Captura de 100 fotos variando altitude de 2.00m até 1.50m (SEM rotação de Yaw).")
    parser.add_argument("--conexao", default=DEFAULT_CONEXAO, help="Porta MAVLink de conexão")
    parser.add_argument("--alt-topo", type=float, default=ALT_TOPO, help="Altitude inicial máxima (padrão: 2.00m)")
    parser.add_argument("--alt-divisao", type=float, default=ALT_DIVISAO, help="Altitude de transição para a zona crítica (padrão: 1.60m)")
    parser.add_argument("--alt-min-yaw", type=float, default=ALT_YAW_MIN, help="Altitude mínima da zona crítica (padrão: 1.48m)")
    parser.add_argument("--alt-max-yaw", type=float, default=ALT_YAW_MAX, help="Altitude de retorno da zona crítica (padrão: 1.55m)")
    parser.add_argument("--fotos-alta", type=int, default=DEFAULT_FOTOS_ALTA, help="Fotos na faixa alta 2.0m->1.6m (padrão: 30)")
    parser.add_argument("--fotos-yaw-1", type=int, default=DEFAULT_FOTOS_YAW_1, help="Fotos descendo na zona crítica 1.6m->1.48m (padrão: 35)")
    parser.add_argument("--fotos-yaw-2", type=int, default=DEFAULT_FOTOS_YAW_2, help="Fotos subindo na zona crítica 1.48m->1.55m (padrão: 35)")
    parser.add_argument("--vel-alta", type=float, default=DEFAULT_VELOCIDADE_Z_ALTA, help="Velocidade Z na faixa alta (padrão: 0.05 m/s)")
    parser.add_argument("--vel-fina", type=float, default=DEFAULT_VELOCIDADE_Z_FINA, help="Velocidade Z na zona crítica (padrão: 0.025 m/s)")
    parser.add_argument("--gimbal-yaw", type=float, default=DEFAULT_GIMBAL_YAW, help="Yaw inicial do Gimbal (padrão: -90°)")
    parser.add_argument("--saida", default=DEFAULT_PASTA_SAIDA, help="Pasta de destino das imagens")
    args = parser.parse_args()

    total_fotos = args.fotos_alta + args.fotos_yaw_1 + args.fotos_yaw_2
    total_fotos_yaw = args.fotos_yaw_1 + args.fotos_yaw_2

    pasta_saida = os.path.abspath(args.saida)
    os.makedirs(pasta_saida, exist_ok=True)
    contador_foto = obter_proximo_numero_captura(pasta_saida)

    print("=" * 75)
    print(" CAPTURA FOCADA: AJUSTE DE YAW (SEM GIRO - VARIAÇÃO PURA DE ALTITUDE)")
    print("=" * 75)
    print(f" Proa / Rotação           : RIGOROSAMENTE FIXA NO NORTE (0.0°) - SEM GIRO")
    print(f" Faixa 1 (Descida Alta)   : {args.alt_topo:.2f}m -> {args.alt_divisao:.2f}m ({args.fotos_alta} fotos)")
    print(f" Faixa 2 (Zona Yaw Desc)  : {args.alt_divisao:.2f}m -> {args.alt_min_yaw:.2f}m ({args.fotos_yaw_1} fotos - Cruzando 1.50m)")
    print(f" Faixa 3 (Zona Yaw Sobe)  : {args.alt_min_yaw:.2f}m -> {args.alt_max_yaw:.2f}m ({args.fotos_yaw_2} fotos - Entorno de 1.50m)")
    print(f" Total de Fotos na Alt 1.5m: {total_fotos_yaw} fotos")
    print(f" Meta Total de Fotos      : {total_fotos} fotos")
    print(f" Pasta de Destino         : {pasta_saida}")
    print(f" Primeira Imagem          : captura_{contador_foto:03d}_...")
    print("=" * 75)

    if rclpy is None:
        print("\n[ERRO] ROS 2 (rclpy) não foi encontrado! Ative com 'source /opt/ros/jazzy/setup.bash'\n")
        sys.exit(1)

    if not vehicle_module_ok:
        print("\n[ERRO] DroneKit ou pymavlink não estão disponíveis!\n")
        sys.exit(1)

    MISSAO_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../missao"))
    if MISSAO_DIR not in sys.path:
        sys.path.append(MISSAO_DIR)

    from controle_voo.takeoff import arm_and_takeoff

    # 1. Inicia ROS 2
    rclpy.init()
    node_camera = ReceptorCameraROS()

    # 2. Conecta ao Drone
    print(f"\n[*] Conectando ao drone em {args.conexao}...")
    try:
        vehicle = connect(args.conexao, wait_ready=False, heartbeat_timeout=15)
        if not aguardar_atributos(vehicle):
            print("[!] ERRO: Telemetria não respondeu.")
            sys.exit(1)
    except Exception as e:
        print(f"[!] ERRO AO CONECTAR: {e}")
        sys.exit(1)

    print("[+] Conectado com sucesso!")

    # Trava WP_YAW_BEHAVIOR = 0
    try:
        vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
    except Exception:
        pass

    yaw_gimbal_atual = args.gimbal_yaw
    ajustar_gimbal_completo(vehicle, pitch_deg=-90.0, yaw_deg=yaw_gimbal_atual)

    # 3. Decolagem / Alinhamento Inicial
    alt_atual = vehicle.location.global_relative_frame.alt
    if alt_atual < 0.5:
        print(f"[!] Decolando para {args.alt_topo:.2f}m...")
        arm_and_takeoff(vehicle, args.alt_topo)

    # Move e paira no centro da armadilha a 2.0m com proa Norte (0.0°)
    navegar_e_estabilizar(
        vehicle,
        alvo_x=ALVO_NORTE_NED,
        alvo_y=ALVO_LESTE_NED,
        alt_desejada=args.alt_topo,
        node_camera=node_camera
    )

    nome_janela = "Captura Foco Yaw - Variacao de Altura (Sem Giro)"
    cv2.namedWindow(nome_janela, cv2.WINDOW_AUTOSIZE)

    # 4. Aguarda confirmação interativa
    yaw_gimbal_atual = aguardar_confirmacao_usuario(
        vehicle, node_camera, nome_janela, yaw_gimbal_atual, altura=args.alt_topo
    )

    # =========================================================================
    # DEFINIÇÃO DAS ETAPAS VERTICAIS (SEM GIRO DE YAW)
    # =========================================================================
    etapas = [
        {
            'nome': "ETAPA 1/3: APROXIMACAO SUPERIOR",
            'desc': f"{args.alt_topo:.2f}m -> {args.alt_divisao:.2f}m",
            'alt_ini': args.alt_topo,
            'alt_fim': args.alt_divisao,
            'meta_fotos': args.fotos_alta,
            'vel_z': args.vel_alta
        },
        {
            'nome': "ETAPA 2/3: ZONA CRITICA DE YAW (DESCIDA)",
            'desc': f"{args.alt_divisao:.2f}m -> {args.alt_min_yaw:.2f}m (Cruzando 1.50m)",
            'alt_ini': args.alt_divisao,
            'alt_fim': args.alt_min_yaw,
            'meta_fotos': args.fotos_yaw_1,
            'vel_z': args.vel_fina
        },
        {
            'nome': "ETAPA 3/3: ZONA CRITICA DE YAW (SUBIDA)",
            'desc': f"{args.alt_min_yaw:.2f}m -> {args.alt_max_yaw:.2f}m (Entorno 1.50m)",
            'alt_ini': args.alt_min_yaw,
            'alt_fim': args.alt_max_yaw,
            'meta_fotos': args.fotos_yaw_2,
            'vel_z': args.vel_fina
        }
    ]

    fotos_salvas = 0
    em_pausa = False

    try:
        for idx_e, etapa in enumerate(etapas):
            print(f"\n[{etapa['nome']}] {etapa['desc']} | Meta: {etapa['meta_fotos']} fotos | Vel: {etapa['vel_z']:.3f} m/s...")
            
            descendo = etapa['alt_fim'] < etapa['alt_ini']
            alt_comando = etapa['alt_ini']
            span_alt = abs(etapa['alt_fim'] - etapa['alt_ini'])
            passo_alt = span_alt / float(max(1, etapa['meta_fotos']))
            ref_alt_foto = alt_comando
            fotos_etapa = 0

            t_anterior = time.time()

            while fotos_etapa < etapa['meta_fotos']:
                t_agora = time.time()
                dt = t_agora - t_anterior
                t_anterior = t_agora

                if rclpy is not None:
                    rclpy.spin_once(node_camera, timeout_sec=0.01)

                if not em_pausa:
                    if descendo:
                        alt_comando = max(etapa['alt_fim'], alt_comando - etapa['vel_z'] * dt)
                    else:
                        alt_comando = min(etapa['alt_fim'], alt_comando + etapa['vel_z'] * dt)

                # Mantém rigorosamente Yaw fixo em 0.0 rad (Norte) - SEM GIRO!
                comandar_posicao_absoluta_local(
                    vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, -alt_comando, yaw_rad=0.0
                )

                alt_real = vehicle.location.global_relative_frame.alt

                # HUD Gráfico
                if node_camera.ultimo_frame is not None:
                    display = cv2.resize(node_camera.ultimo_frame, (960, 540))
                    cv2.rectangle(display, (0, 0), (960, 68), (20, 20, 20), -1)
                    cor = (0, 165, 255) if em_pausa else (0, 255, 0)
                    cv2.putText(display, f"[{etapa['nome']}] Total: {fotos_salvas}/{total_fotos} | Etapa: {fotos_etapa}/{etapa['meta_fotos']}", 
                                (15, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.50, cor, 2)
                    cv2.putText(display, f"Alt Real: {alt_real:.2f}m (Cmd: {alt_comando:.2f}m -> Fim: {etapa['alt_fim']:.2f}m) | Proa: 0.0 deg (FIXA - SEM GIRO)", 
                                (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1)
                    cv2.putText(display, "[ESPACO]: Pausa | [C]: Foto Manual | [Q]: Cancelar", 
                                (15, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (200, 200, 200), 1)
                    cv2.drawMarker(display, (480, 270), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)
                    cv2.imshow(nome_janela, display)

                tecla = cv2.waitKey(15) & 0xFF
                if tecla == 32:
                    em_pausa = not em_pausa
                    t_anterior = time.time()
                    print(f"[!] {'PAUSADO' if em_pausa else 'RETOMADO'}")
                elif tecla in [ord('q'), ord('Q'), 27]:
                    raise KeyboardInterrupt

                # Disparo automático baseado no deslocamento vertical
                tirar_foto = False
                chegou_ao_fim = (descendo and alt_comando <= etapa['alt_fim'] + 0.003) or ((not descendo) and alt_comando >= etapa['alt_fim'] - 0.003)

                if tecla in [ord('c'), ord('C')]:
                    tirar_foto = True
                elif not em_pausa and node_camera.ultimo_frame is not None:
                    if abs(ref_alt_foto - alt_comando) >= passo_alt or (chegou_ao_fim and fotos_etapa < etapa['meta_fotos']):
                        ref_alt_foto = alt_comando
                        tirar_foto = True

                if tirar_foto and node_camera.ultimo_frame is not None:
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
                    nome = f"captura_{contador_foto:03d}_{ts}.jpg"
                    caminho = os.path.join(pasta_saida, nome)
                    cv2.imwrite(caminho, node_camera.ultimo_frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
                    fotos_salvas += 1
                    contador_foto += 1
                    fotos_etapa += 1
                    print(f" -> Foto {fotos_salvas}/{total_fotos} (Etapa {fotos_etapa}/{etapa['meta_fotos']}) salva em Alt={alt_real:.2f}m")

        # Conclusão e retorno seguro
        print("\n" + "=" * 75)
        print(f"[+] SUCESSO! TODAS AS 100 FOTOS CAPTURADAS COM SUCESSO!")
        print(f"    * Proa: Estritamente fixa em 0.0° (sem rotação)")
        print(f"    * 30 fotos na faixa alta (2.00m -> 1.60m)")
        print(f"    * 70 fotos concentradas na faixa de ajuste de Yaw (1.60m <-> 1.48m)")
        print(f"    * Salvas em: {pasta_saida}")
        print("=" * 75)

        print("\n[*] Retornando o drone com segurança para 2.00m...")
        navegar_e_estabilizar(
            vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, alt_desejada=args.alt_topo, node_camera=node_camera
        )

    except KeyboardInterrupt:
        print("\n[!] Operação interrompida pelo usuário.")
    finally:
        print("\n[*] Encerrando nós e conexões...")
        cv2.destroyAllWindows()
        if rclpy is not None and rclpy.ok():
            node_camera.destroy_node()
            rclpy.shutdown()
        vehicle.close()
        print("[+] Finalizado com sucesso!")


if __name__ == "__main__":
    main()
