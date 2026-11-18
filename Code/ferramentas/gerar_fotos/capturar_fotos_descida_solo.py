#!/usr/bin/env python3
"""
=============================================================================
Captura Automatizada de Fotos em Varredura Vertical sobre a Armadilha (0.35m <-> 0.20m)
=============================================================================

Sequência autônoma executada por este script:
1. Conecta ao ArduPilot SITL (DroneKit/MAVLink), ao tópico ROS 2 (/camera/image)
   e ao tópico de poses reais do Gazebo (/world/iris_runway/pose/info).
2. Configura trava de Yaw (WP_YAW_BEHAVIOR = 0) e aponta o gimbal em Nadir
   (-90° pitch, -90° yaw por padrão, idêntico à missão principal).
3. Decola (se estiver no solo) e voa até ficar exatamente acima da armadilha
   a uma altura de 0.50 m, realizando ajuste fino em malha fechada (XY e Yaw)
   com base na pose real do Gazebo.
4. Estando estabilizado e centralizado a 0.50 m, desce verticalmente até 0.35 m
   mantendo o alinhamento exato sobre a armadilha.
5. A partir de 0.35 m, realiza ciclos contínuos de descida e subida entre
   0.35 m e 0.20 m ("indo e voltando", com limitação estrita para não passar
   acima de 0.35 m nem abaixo de 0.20 m), mantendo o drone centralizado sobre
   a armadilha e capturando 200 fotos ao longo da faixa.
6. Ao concluir as 200 fotos, sobe com segurança de volta para 0.50 m acima da
   armadilha e salva um registro CSV com altitude e erro XY de cada foto.

Uso:
  source /opt/ros/jazzy/setup.bash
  python3 capturar_fotos_descida_solo.py
=============================================================================
"""

# --- COMPATIBILIDADE DRONEKIT PYTHON 3.10+ ---
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------

import argparse
import csv
from datetime import datetime
import glob
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

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
    from dronekit import connect, VehicleMode
    from pymavlink import mavutil
except ImportError:
    vehicle_module_ok = False


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MISSAO_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../missao"))
if MISSAO_DIR not in sys.path:
    sys.path.insert(0, MISSAO_DIR)

# =============================================================================
# CONSTANTES E CONFIGURAÇÕES PADRÃO
# =============================================================================
DEFAULT_CONEXAO = "udp:127.0.0.1:14550"
DEFAULT_WORLD_GAZEBO = "iris_runway"
DEFAULT_TOPICO_CAMERA = "/camera/image"

# Fallback caso o Gazebo não esteja acessível via CLI (tcc.sdf)
FALLBACK_ARMADILHA_X_GZ = 50.0  # Leste
FALLBACK_ARMADILHA_Y_GZ = 0.0   # Norte
# Altura de base_link sobre o plano Z=0 quando os pés do drone tocam o solo (STL)
DEFAULT_Z_SOLO_DRONE = 0.4048

ALT_INICIAL_ALVO = 0.50
ALT_FAIXA_MAX = 0.35
ALT_FAIXA_MIN = 0.20
ALT_TRANSITO_PADRAO = 1.00
DEFAULT_TOTAL_FOTOS = 200
DEFAULT_CICLOS = 5                  # 5 descidas + 5 subidas = 10 trechos de 20 fotos

DEFAULT_VEL_DESCIDA_PREPARO = 0.04  # m/s (0.50m -> 0.35m)
DEFAULT_VEL_VARREDURA_Z = 0.025     # m/s (0.35m <-> 0.20m: ~1.25 mm/frame @ 20 FPS)
DEFAULT_GIMBAL_YAW = -90.0

DEFAULT_TOL_XY_M = 0.012            # 1.2 cm para considerar "exatamente acima"
DEFAULT_TOL_XY_DESCIDA_M = 0.020    # 2.0 cm limite máximo antes de pausar movimento vertical
DEFAULT_TOL_ALT_M = 0.012           # 1.2 cm de erro vertical para estabilização
DEFAULT_TOL_YAW_DEG = 2.0

DEFAULT_PASTA_SAIDA = os.path.abspath(os.path.join(SCRIPT_DIR, "fotos"))


def normalizar_angulo_rad(ang):
    """Normaliza ângulo em radianos para o intervalo [-pi, pi]."""
    return (ang + math.pi) % (2.0 * math.pi) - math.pi


def clamp(valor, minimo, maximo):
    """Restringe um valor ao intervalo [minimo, maximo]."""
    return max(minimo, min(maximo, valor))


def quaternion_para_yaw_rad(ori):
    """Extrai o ângulo Yaw (rad) de um quaternion {x, y, z, w} no referencial ENU."""
    x = ori.get("x", 0.0)
    y = ori.get("y", 0.0)
    z = ori.get("z", 0.0)
    w = ori.get("w", 1.0)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def obter_proximo_numero_captura(pasta_destino):
    """Descobre o próximo índice sequencial de captura disponível na pasta."""
    os.makedirs(pasta_destino, exist_ok=True)
    arquivos = glob.glob(os.path.join(pasta_destino, "*.jpg")) + glob.glob(
        os.path.join(pasta_destino, "*.png")
    )
    maior_num = 0
    padrao = re.compile(r"captura_(\d+)_")
    for arq in arquivos:
        match = padrao.search(os.path.basename(arq))
        if match:
            maior_num = max(maior_num, int(match.group(1)))
    return maior_num + 1


def aguardar_atributos(vehicle, timeout=30):
    """Aguarda telemetria básica do ArduPilot ficar disponível."""
    inicio = time.time()
    print("[*] Aguardando leitura da telemetria básica do ArduPilot...")
    while time.time() - inicio < timeout:
        if (
            vehicle.mode.name
            and vehicle.location.global_relative_frame
            and vehicle.location.global_relative_frame.alt is not None
            and vehicle.is_armable is not None
        ):
            return True
        time.sleep(0.5)
    return False


class MonitorPoseGazebo:
    """
    Lê continuamente as poses absolutas de `drone_coaxial` e `armadilha_alvo`
    via streaming de `gz topic -e -t /world/<world>/pose/info`.
    """

    def __init__(self, world_name=DEFAULT_WORLD_GAZEBO):
        self.world_name = world_name
        self.topico = f"/world/{world_name}/pose/info"
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._proc = None
        self._thread = None
        self._poses = {}
        self._ultimo_update_drone = 0.0
        self._ultimo_update_armadilha = 0.0

    def iniciar(self):
        """Inicia a thread de leitura contínua do tópico de poses do Gazebo."""
        gz_bin = shutil.which("gz")
        if gz_bin is None:
            return False

        cmd = [gz_bin, "topic", "-e", "-t", self.topico]
        stdbuf_bin = shutil.which("stdbuf")
        if stdbuf_bin is not None:
            cmd = [stdbuf_bin, "-oL"] + cmd

        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        except Exception:
            self._proc = None
            return False

        self._thread = threading.Thread(target=self._loop_leitura, daemon=True)
        self._thread.start()
        return True

    def _loop_leitura(self):
        depth = 0
        in_pose = False
        section = None
        nome_atual = None
        pos_atual = None
        ori_atual = None

        stdout = self._proc.stdout if self._proc is not None else None
        if stdout is None:
            return

        for raw_line in stdout:
            if self._stop_event.is_set():
                break

            line = raw_line.strip()
            if not line:
                continue

            if line.startswith("pose {"):
                in_pose = True
                depth = 1
                section = None
                nome_atual = None
                pos_atual = {"x": 0.0, "y": 0.0, "z": 0.0}
                ori_atual = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                continue

            if not in_pose:
                continue

            if line == "}":
                depth -= 1
                if depth == 1:
                    section = None
                elif depth == 0:
                    if nome_atual in ("drone_coaxial", "armadilha_alvo"):
                        agora_pose = time.time()
                        with self._lock:
                            self._poses[nome_atual] = {
                                "position": pos_atual,
                                "orientation": ori_atual,
                            }
                            if nome_atual == "drone_coaxial":
                                self._ultimo_update_drone = agora_pose
                            else:
                                self._ultimo_update_armadilha = agora_pose
                    in_pose = False
                continue

            if line.endswith("{"):
                depth += 1
                if depth == 2:
                    if line.startswith("position"):
                        section = "position"
                    elif line.startswith("orientation"):
                        section = "orientation"
                continue

            if ":" in line:
                chave, valor = line.split(":", 1)
                chave = chave.strip()
                valor = valor.strip().strip('"')
                try:
                    if depth == 1 and chave == "name":
                        nome_atual = valor
                    elif depth == 2 and section == "position" and chave in pos_atual:
                        pos_atual[chave] = float(valor)
                    elif depth == 2 and section == "orientation" and chave in ori_atual:
                        ori_atual[chave] = float(valor)
                except ValueError:
                    pass

    def aguardar_poses(self, timeout=5.0):
        """Aguarda o recebimento inicial das poses do drone e da armadilha."""
        inicio = time.time()
        while time.time() - inicio < timeout:
            estado = self.obter_estado(max_idade_s=2.0)
            if estado is not None:
                return estado
            time.sleep(0.1)
        return None

    def obter_estado(self, max_idade_s=1.0):
        """Retorna (pos_drone, ori_drone, pos_armadilha, ori_armadilha) se atualizados."""
        with self._lock:
            agora = time.time()
            if (
                "drone_coaxial" in self._poses
                and "armadilha_alvo" in self._poses
                and (agora - self._ultimo_update_drone) <= max_idade_s
                and self._ultimo_update_armadilha > 0.0
            ):
                drone = self._poses["drone_coaxial"]
                arm = self._poses["armadilha_alvo"]
                return (
                    dict(drone["position"]),
                    dict(drone["orientation"]),
                    dict(arm["position"]),
                    dict(arm["orientation"]),
                )
        return None

    def parar(self):
        """Encerra o processo filho e a thread de leitura."""
        self._stop_event.set()
        if self._proc is not None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=1.5)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            self._proc = None


BaseNode = Node if Node is not None else object


class ReceptorCameraROS(BaseNode):
    """Nó ROS 2 que mantém em memória o frame mais recente de /camera/image."""

    def __init__(self, topico=DEFAULT_TOPICO_CAMERA):
        if Node is not None:
            super().__init__("receptor_camera_descida_solo")
        self.bridge = CvBridge() if CvBridge is not None else None
        self._lock = threading.Lock()
        self._ultimo_frame = None
        self._frame_seq = 0
        self._ultimo_ts = 0.0

        if Node is not None:
            self.subscription = self.create_subscription(
                Image, topico, self._image_callback, 10
            )

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._ultimo_frame = frame
                self._frame_seq += 1
                self._ultimo_ts = time.time()
        except Exception:
            pass

    def obter_frame(self):
        """Retorna (copia_frame, seq, timestamp) ou (None, 0, 0.0)."""
        with self._lock:
            if self._ultimo_frame is None:
                return None, self._frame_seq, self._ultimo_ts
            return self._ultimo_frame.copy(), self._frame_seq, self._ultimo_ts


class ExecutorROSBG:
    """Executa rclpy.spin_once em thread dedicada para manter os frames sempre frescos."""

    def __init__(self, node):
        self.node = node
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def iniciar(self):
        self._thread.start()

    def _run(self):
        while not self._stop.is_set() and rclpy is not None and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            except Exception:
                time.sleep(0.02)

    def parar(self):
        self._stop.set()
        self._thread.join(timeout=1.5)


class ControladorPrecisaoArmadilha:
    """
    Controlador em malha fechada que usa a verdade terrestre do Gazebo (com
    fallback automático para LOCAL_NED) para posicionar e manter o drone
    milimetricamente acima da armadilha e controlar sua altitude exata.
    """

    def __init__(
        self,
        vehicle,
        monitor_gz,
        z_solo_drone=DEFAULT_Z_SOLO_DRONE,
        manter_yaw=False,
        offset_norte=0.0,
        offset_leste=0.0,
    ):
        self.vehicle = vehicle
        self.monitor_gz = monitor_gz
        self.z_solo_drone = z_solo_drone
        self.manter_yaw = manter_yaw
        self.offset_norte = offset_norte
        self.offset_leste = offset_leste

        self.bias_n = 0.0
        self.bias_e = 0.0
        self.bias_alt = 0.0
        self.bias_yaw = None
        self.bias_inicializado = False

        self.trim_n = 0.0
        self.trim_e = 0.0
        self.trim_alt = 0.0

        self.yaw_inicial_ned = self._ler_yaw_ned()
        self._ultimo_tempo = time.time()

    def _ler_yaw_ned(self):
        try:
            if self.vehicle.attitude and self.vehicle.attitude.yaw is not None:
                return float(self.vehicle.attitude.yaw)
        except Exception:
            pass
        return 0.0

    def _ler_local_ned(self):
        try:
            loc = self.vehicle.location.local_frame
            if (
                loc is not None
                and loc.north is not None
                and loc.east is not None
                and loc.down is not None
            ):
                return float(loc.north), float(loc.east), -float(loc.down)
        except Exception:
            pass
        alt_rel = 0.0
        try:
            if self.vehicle.location.global_relative_frame.alt is not None:
                alt_rel = float(self.vehicle.location.global_relative_frame.alt)
        except Exception:
            pass
        return None, None, alt_rel

    def enviar_setpoint_local_ned(self, norte, leste, alt_ekf, yaw_rad):
        """Envia SET_POSITION_TARGET_LOCAL_NED com posição XYZ e Yaw explícitos."""
        ts = getattr(self.vehicle, "_master", None)
        target_sys = ts.target_system if ts and ts.target_system else 1
        target_comp = ts.target_component if ts and ts.target_component else 1

        # Usa X, Y, Z e YAW absoluto (ignora velocidades, acelerações e yaw_rate)
        typemask = 0b0000100111111000
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            target_sys,
            target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            typemask,
            float(norte),
            float(leste),
            -float(alt_ekf),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(yaw_rad),
            0.0,
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def atualizar_e_comandar(self, alvo_alt_m, usar_trim_xy=True, usar_trim_alt=True):
        """Calcula o erro atual e envia o setpoint corrigido para o ArduPilot."""
        agora = time.time()
        dt = clamp(agora - self._ultimo_tempo, 0.005, 0.20)
        self._ultimo_tempo = agora

        loc_n, loc_e, alt_ekf = self._ler_local_ned()
        yaw_ekf = self._ler_yaw_ned()
        estado_gz = self.monitor_gz.obter_estado(max_idade_s=1.0) if self.monitor_gz else None

        if estado_gz is not None:
            pos_d, ori_d, pos_a, ori_a = estado_gz
            alvo_n_gz = pos_a["y"] + self.offset_norte
            alvo_e_gz = pos_a["x"] + self.offset_leste
            alt_real = pos_d["z"] - self.z_solo_drone

            err_n = alvo_n_gz - pos_d["y"]
            err_e = alvo_e_gz - pos_d["x"]
            dist_xy = math.hypot(err_n, err_e)
            err_alt = alvo_alt_m - alt_real

            yaw_drone_gz = quaternion_para_yaw_rad(ori_d)
            yaw_arm_gz = quaternion_para_yaw_rad(ori_a)

            vel_horiz = 0.0
            try:
                if self.vehicle.groundspeed is not None:
                    vel_horiz = float(self.vehicle.groundspeed)
            except Exception:
                vel_horiz = 0.0

            if loc_n is not None and loc_e is not None:
                obs_bias_n = loc_n - pos_d["y"]
                obs_bias_e = loc_e - pos_d["x"]
                obs_bias_alt = alt_ekf - alt_real
                obs_bias_yaw = normalizar_angulo_rad(yaw_ekf + yaw_drone_gz)

                if not self.bias_inicializado:
                    self.bias_n = obs_bias_n
                    self.bias_e = obs_bias_e
                    self.bias_alt = obs_bias_alt
                    self.bias_yaw = obs_bias_yaw
                    self.bias_inicializado = True
                elif dist_xy < 1.5 and vel_horiz < 0.35:
                    alpha = 0.08
                    alpha_alt = 0.08 if usar_trim_alt else 0.02
                    self.bias_n = (1.0 - alpha) * self.bias_n + alpha * obs_bias_n
                    self.bias_e = (1.0 - alpha) * self.bias_e + alpha * obs_bias_e
                    self.bias_alt = (1.0 - alpha_alt) * self.bias_alt + alpha_alt * obs_bias_alt
                    diff_byaw = normalizar_angulo_rad(obs_bias_yaw - self.bias_yaw)
                    self.bias_yaw = normalizar_angulo_rad(self.bias_yaw + alpha * diff_byaw)

            if usar_trim_xy and dist_xy < 0.60:
                ki_xy = 0.35
                self.trim_n = clamp(self.trim_n + ki_xy * err_n * dt, -0.20, 0.20)
                self.trim_e = clamp(self.trim_e + ki_xy * err_e * dt, -0.20, 0.20)

            if usar_trim_alt and abs(err_alt) < 0.25:
                ki_z = 0.35
                self.trim_alt = clamp(self.trim_alt + ki_z * err_alt * dt, -0.15, 0.15)

            if self.manter_yaw or self.bias_yaw is None:
                yaw_cmd = self.yaw_inicial_ned
                err_yaw_deg = math.degrees(normalizar_angulo_rad(self.yaw_inicial_ned - yaw_ekf))
            else:
                yaw_cmd = normalizar_angulo_rad(self.bias_yaw - yaw_arm_gz)
                err_yaw_deg = math.degrees(normalizar_angulo_rad(yaw_arm_gz - yaw_drone_gz))

            cmd_n = alvo_n_gz + self.bias_n + self.trim_n
            cmd_e = alvo_e_gz + self.bias_e + self.trim_e
            cmd_alt = alvo_alt_m + self.bias_alt + self.trim_alt
            self.enviar_setpoint_local_ned(cmd_n, cmd_e, cmd_alt, yaw_cmd)

            return {
                "usando_gazebo": True,
                "alt_real": alt_real,
                "alt_ardupilot": alt_ekf,
                "err_n": err_n,
                "err_e": err_e,
                "dist_xy": dist_xy,
                "err_yaw_deg": err_yaw_deg,
                "pos_drone": pos_d,
                "pos_armadilha": pos_a,
            }

        # Fallback: usa apenas coordenadas LOCAL_NED / alt relativo do ArduPilot
        alvo_n = FALLBACK_ARMADILHA_Y_GZ + self.offset_norte
        alvo_e = FALLBACK_ARMADILHA_X_GZ + self.offset_leste
        err_n = (alvo_n - loc_n) if loc_n is not None else 999.0
        err_e = (alvo_e - loc_e) if loc_e is not None else 999.0
        dist_xy = math.hypot(err_n, err_e) if loc_n is not None else 999.0
        yaw_cmd = self.yaw_inicial_ned
        err_yaw_deg = math.degrees(normalizar_angulo_rad(yaw_cmd - yaw_ekf))
        self.enviar_setpoint_local_ned(alvo_n, alvo_e, alvo_alt_m, yaw_cmd)

        return {
            "usando_gazebo": False,
            "alt_real": alt_ekf,
            "alt_ardupilot": alt_ekf,
            "err_n": err_n,
            "err_e": err_e,
            "dist_xy": dist_xy,
            "err_yaw_deg": err_yaw_deg,
            "pos_drone": None,
            "pos_armadilha": None,
        }


def exibir_hud_e_ler_tecla(
    mostrar_janela,
    nome_janela,
    frame,
    etapa_titulo,
    telemetry,
    alvo_alt_m,
    fotos_salvas=0,
    total_fotos=0,
    pausado=False,
):
    """Atualiza a janela OpenCV com HUD informativo e verifica se usuário pressionou Q/ESC."""
    if not mostrar_janela:
        return None

    if frame is not None:
        disp = cv2.resize(frame, (960, 540))
    else:
        disp = np.zeros((540, 960, 3), dtype=np.uint8)
        cv2.putText(
            disp,
            "Aguardando frames em /camera/image...",
            (180, 270),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

    cv2.rectangle(disp, (0, 0), (960, 74), (18, 18, 18), -1)
    cor_titulo = (0, 165, 255) if pausado else (0, 255, 0)
    progresso = f" | Fotos: {fotos_salvas}/{total_fotos}" if total_fotos > 0 else ""
    cv2.putText(
        disp,
        f"{etapa_titulo}{progresso}",
        (14, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        cor_titulo,
        2,
    )

    fonte_dados = "GAZEBO" if telemetry["usando_gazebo"] else "EKF-FALLBACK"
    dist_mm = telemetry["dist_xy"] * 1000.0
    cv2.putText(
        disp,
        (
            f"[{fonte_dados}] Alt: {telemetry['alt_real']:.3f}m (Alvo: {alvo_alt_m:.3f}m) | "
            f"Erro XY: {dist_mm:.1f}mm | Yaw: {telemetry['err_yaw_deg']:+.1f} deg"
        ),
        (14, 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        disp,
        "[Q / ESC]: Abortar com seguranca",
        (14, 66),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (190, 190, 190),
        1,
    )
    cv2.drawMarker(disp, (480, 270), (0, 255, 255), cv2.MARKER_CROSS, 22, 1)
    cv2.imshow(nome_janela, disp)
    return cv2.waitKey(1) & 0xFF


def aguardar_frame_novo(node_camera, seq_anterior, timeout=0.35):
    """Aguarda um novo frame da câmera ROS 2 posterior a `seq_anterior`."""
    inicio = time.time()
    while time.time() - inicio < timeout:
        frame, seq, ts = node_camera.obter_frame()
        if frame is not None and seq > seq_anterior:
            return frame, seq, ts
        time.sleep(0.01)
    return node_camera.obter_frame()


def executar_etapa_alinhamento(
    controlador,
    node_camera,
    mostrar_janela,
    nome_janela,
    alvo_alt_m,
    tol_xy_m,
    tol_alt_m,
    tol_yaw_deg,
    tempo_sustentado_s=1.2,
    timeout_s=45.0,
    titulo="ETAPA 1: ALINHANDO EXATAMENTE ACIMA DA ARMADILHA",
    alt_transito_m=ALT_TRANSITO_PADRAO,
):
    """
    Navega até a armadilha (mantendo altitude de trânsito segura enquanto distante)
    e depois estabiliza na altitude desejada dentro das tolerâncias milimétricas.
    """
    inicio = time.time()
    inicio_ok = None
    ultimo_log = 0.0
    alt_cmd_etapa = max(alvo_alt_m, alt_transito_m)
    t_ant = inicio

    while True:
        agora_loop = time.time()
        dt = clamp(agora_loop - t_ant, 0.005, 0.15)
        t_ant = agora_loop

        # Mantém altitude de trânsito segura até chegar perto do centro da armadilha (<= 0.25m);
        # só então desce suavemente até 0.50m e executa o alinhamento fino milimétrico.
        estado_gz = controlador.monitor_gz.obter_estado(max_idade_s=1.0) if controlador.monitor_gz else None
        if estado_gz is not None:
            pos_d, _, pos_a, _ = estado_gz
            dist_bruta = math.hypot(
                (pos_a["x"] + controlador.offset_leste) - pos_d["x"],
                (pos_a["y"] + controlador.offset_norte) - pos_d["y"],
            )
        else:
            loc_n, loc_e, _ = controlador._ler_local_ned()
            if loc_n is not None and loc_e is not None:
                dist_bruta = math.hypot(
                    (FALLBACK_ARMADILHA_Y_GZ + controlador.offset_norte) - loc_n,
                    (FALLBACK_ARMADILHA_X_GZ + controlador.offset_leste) - loc_e,
                )
            else:
                dist_bruta = 999.0

        perto_alvo = dist_bruta <= 0.25
        if perto_alvo and alt_cmd_etapa > alvo_alt_m:
            alt_cmd_etapa = max(alvo_alt_m, alt_cmd_etapa - 0.15 * dt)

        tel = controlador.atualizar_e_comandar(
            alt_cmd_etapa,
            usar_trim_xy=(dist_bruta <= 0.8),
            usar_trim_alt=perto_alvo,
        )

        frame, _, _ = node_camera.obter_frame()
        tecla = exibir_hud_e_ler_tecla(
            mostrar_janela, nome_janela, frame, titulo, tel, alt_cmd_etapa
        )
        if tecla in (ord("q"), ord("Q"), 27):
            raise KeyboardInterrupt

        erro_alt = abs(tel["alt_real"] - alvo_alt_m)
        ok_xy = tel["dist_xy"] <= tol_xy_m
        ok_alt = erro_alt <= tol_alt_m
        ok_yaw = abs(tel["err_yaw_deg"]) <= tol_yaw_deg

        agora = time.time()
        if perto_alvo and ok_xy and ok_alt and ok_yaw:
            if inicio_ok is None:
                inicio_ok = agora
            elif (agora - inicio_ok) >= tempo_sustentado_s:
                print(
                    f"\n[+] ESTABILIZADO @ {tel['alt_real']:.3f}m | "
                    f"Erro XY={tel['dist_xy']*1000:.1f}mm | Yaw={tel['err_yaw_deg']:+.1f}°"
                )
                return tel
        else:
            inicio_ok = None

        if agora - ultimo_log >= 0.5:
            ultimo_log = agora
            sust_s = (agora - inicio_ok) if inicio_ok is not None else 0.0
            sys.stdout.write(
                f"\r -> [{titulo[:28]}] DistXY={tel['dist_xy']*1000:6.1f}mm | "
                f"Alt={tel['alt_real']:.3f}m (Alvo {alvo_alt_m:.2f}m) | "
                f"Yaw={tel['err_yaw_deg']:+5.1f}° | OK={sust_s:.1f}/{tempo_sustentado_s:.1f}s   "
            )
            sys.stdout.flush()

        if agora - inicio >= timeout_s:
            print(
                f"\n[!] Timeout de alinhamento ({timeout_s:.0f}s). "
                f"Estado atual: Alt={tel['alt_real']:.3f}m, Erro XY={tel['dist_xy']*1000:.1f}mm."
            )
            return tel

        time.sleep(0.04)


def descer_para_altitude_alinhado(
    controlador,
    node_camera,
    mostrar_janela,
    nome_janela,
    alt_inicial_m,
    alt_alvo_m,
    vel_descida_ms,
    tol_xy_m,
    tol_xy_pausa_m,
    tol_alt_m,
    tol_yaw_deg,
):
    """
    Desce de `alt_inicial_m` até `alt_alvo_m` mantendo o drone centralizado
    acima da armadilha (pausa a descida caso o erro horizontal ultrapasse `tol_xy_pausa_m`).
    """
    print(f"\n[*] ETAPA 2: Descendo de {alt_inicial_m:.2f}m até {alt_alvo_m:.2f}m centralizado sobre a armadilha...")
    alt_cmd = alt_inicial_m
    t_ant = time.time()
    inicio_estab = None
    ultimo_log = 0.0

    while True:
        agora = time.time()
        dt = clamp(agora - t_ant, 0.005, 0.15)
        t_ant = agora

        tel = controlador.atualizar_e_comandar(alt_cmd, usar_trim_xy=True, usar_trim_alt=True)
        pausado = tel["dist_xy"] > tol_xy_pausa_m

        if not pausado and alt_cmd > alt_alvo_m:
            # Desacelera nos últimos 3 cm e limita o avanço a 1.5 cm abaixo da altitude real
            # para parar em alt_alvo_m sem ultrapassar para baixo
            vel_local = vel_descida_ms * clamp((alt_cmd - alt_alvo_m) / 0.03, 0.35, 1.0)
            alt_cmd = max(alt_alvo_m, alt_cmd - vel_local * dt, tel["alt_real"] - 0.015)

        frame, _, _ = node_camera.obter_frame()
        tecla = exibir_hud_e_ler_tecla(
            mostrar_janela,
            nome_janela,
            frame,
            f"ETAPA 2: DESCENDO PARA {alt_alvo_m:.2f}m",
            tel,
            alt_cmd,
            pausado=pausado,
        )
        if tecla in (ord("q"), ord("Q"), 27):
            raise KeyboardInterrupt

        ok_alt = abs(tel["alt_real"] - alt_alvo_m) <= tol_alt_m
        ok_xy = tel["dist_xy"] <= tol_xy_m
        ok_yaw = abs(tel["err_yaw_deg"]) <= tol_yaw_deg

        if alt_cmd <= alt_alvo_m + 0.001 and ok_alt and ok_xy and ok_yaw:
            if inicio_estab is None:
                inicio_estab = agora
            elif (agora - inicio_estab) >= 0.8:
                print(
                    f"\n[+] ALTITUDE {alt_alvo_m:.2f}m ALCANÇADA! "
                    f"Alt Real={tel['alt_real']:.3f}m | Erro XY={tel['dist_xy']*1000:.1f}mm"
                )
                return tel
        else:
            inicio_estab = None

        if agora - ultimo_log >= 0.4:
            ultimo_log = agora
            estado_str = "RECENTRALIZANDO" if pausado else "DESCENDO"
            sys.stdout.write(
                f"\r -> [{estado_str}] Alt={tel['alt_real']:.3f}m (Cmd={alt_cmd:.3f}m -> {alt_alvo_m:.2f}m) | "
                f"ErroXY={tel['dist_xy']*1000:5.1f}mm   "
            )
            sys.stdout.flush()

        time.sleep(0.04)


def executar_varredura_vertical_com_fotos(
    controlador,
    node_camera,
    mostrar_janela,
    nome_janela,
    pasta_saida,
    contador_inicial,
    alt_max_m,
    alt_min_m,
    total_fotos,
    ciclos,
    vel_captura_ms,
    tol_xy_pausa_m,
):
    """
    Realiza ciclos de descida e subida entre `alt_max_m` (0.35 m) e `alt_min_m` (0.20 m),
    sem passar para cima nem para baixo dos limites, mantendo o drone exatamente acima
    da armadilha e capturando `total_fotos` (200) fotos distribuídas ao longo dos trechos.
    """
    if alt_max_m <= alt_min_m:
        raise ValueError("alt_max_m deve ser maior que alt_min_m.")
    if total_fotos <= 0 or ciclos <= 0:
        raise ValueError("total_fotos e ciclos devem ser positivos.")

    # Margem interna de segurança (3 mm padrão) para evitar overshoot nos pontos de reversão
    inset = min(0.003, (alt_max_m - alt_min_m) * 0.02)
    teto_foto = alt_max_m - inset
    piso_foto = alt_min_m + inset
    teto_cmd = alt_max_m - min(0.002, inset)
    piso_cmd = alt_min_m + min(0.002, inset)

    num_trechos = ciclos * 2
    base_fotos = total_fotos // num_trechos
    resto_fotos = total_fotos % num_trechos

    trechos = []
    for t_idx in range(num_trechos):
        descendo = (t_idx % 2 == 0)
        ciclo_num = (t_idx // 2) + 1
        meta_trecho = base_fotos + (1 if t_idx < resto_fotos else 0)
        if meta_trecho == 0:
            continue

        ini_m = teto_foto if descendo else (piso_foto + 0.001)
        fim_m = piso_foto if descendo else (teto_foto - 0.001)
        if meta_trecho == 1:
            marcos_m = [(ini_m + fim_m) / 2.0]
        else:
            passo = (fim_m - ini_m) / float(meta_trecho - 1)
            marcos_m = [ini_m + k * passo for k in range(meta_trecho)]

        trechos.append(
            {
                "ciclo": ciclo_num,
                "descendo": descendo,
                "dir_tag": "desc" if descendo else "sub",
                "dir_label": "DESCENDO" if descendo else "SUBINDO",
                "alt_fim_cmd": piso_cmd if descendo else teto_cmd,
                "marcos_m": marcos_m,
            }
        )

    print("\n" + "=" * 75)
    print(f" ETAPA 3: VARREDURA VERTICAL ({alt_max_m:.2f}m <-> {alt_min_m:.2f}m) - {total_fotos} FOTOS")
    print("=" * 75)
    print(f" Faixa estrita    : [{alt_min_m:.3f} m, {alt_max_m:.3f} m] (sem ultrapassar limites)")
    print(f" Ciclos / Trechos : {ciclos} ciclos ({len(trechos)} trechos de ida e volta)")
    print(f" Velocidade Z     : {vel_captura_ms:.3f} m/s (~{vel_captura_ms*100:.1f} cm/s)")
    print(f" Pasta de saída   : {pasta_saida}")
    print("=" * 75)

    registros = []
    contador_foto = contador_inicial
    ultimo_seq_salvo = 0

    alt_cmd = clamp(teto_cmd, piso_cmd, teto_cmd)
    t_ant = time.time()

    def salvar_foto(trecho, alt_alvo_foto, tel):
        nonlocal contador_foto, ultimo_seq_salvo
        frame_novo, seq_novo, _ = aguardar_frame_novo(node_camera, ultimo_seq_salvo, timeout=0.25)
        if frame_novo is None:
            return False

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
        alt_cm = max(0.0, tel["alt_real"] * 100.0)
        nome_arq = (
            f"captura_{contador_foto:03d}_c{trecho['ciclo']}_{trecho['dir_tag']}_"
            f"alt_{alt_cm:04.1f}cm_{ts_str}.jpg"
        )
        caminho_foto = os.path.join(pasta_saida, nome_arq)
        cv2.imwrite(caminho_foto, frame_novo, [cv2.IMWRITE_JPEG_QUALITY, 100])

        ultimo_seq_salvo = max(ultimo_seq_salvo, seq_novo)
        contador_foto += 1

        pos_d = tel["pos_drone"] or {}
        pos_a = tel["pos_armadilha"] or {}
        err_n_mm = tel["err_n"] * 1000.0
        err_e_mm = tel["err_e"] * 1000.0
        err_xy_mm = tel["dist_xy"] * 1000.0

        registros.append(
            {
                "arquivo": nome_arq,
                "ciclo": trecho["ciclo"],
                "direcao": trecho["dir_label"],
                "alt_alvo_m": round(alt_alvo_foto, 4),
                "alt_gazebo_m": round(tel["alt_real"], 4),
                "alt_ardupilot_m": round(tel["alt_ardupilot"], 4),
                "erro_norte_mm": round(err_n_mm, 2),
                "erro_leste_mm": round(err_e_mm, 2),
                "erro_xy_mm": round(err_xy_mm, 2),
                "erro_yaw_deg": round(tel["err_yaw_deg"], 2),
                "drone_gz_x": round(pos_d.get("x", 0.0), 5),
                "drone_gz_y": round(pos_d.get("y", 0.0), 5),
                "drone_gz_z": round(pos_d.get("z", 0.0), 5),
                "armadilha_gz_x": round(pos_a.get("x", 0.0), 5),
                "armadilha_gz_y": round(pos_a.get("y", 0.0), 5),
            }
        )

        print(
            f"[FOTO {len(registros):3d}/{total_fotos}] C{trecho['ciclo']} {trecho['dir_label']:8s} | "
            f"Alt Real: {tel['alt_real']:.3f}m (Alvo {alt_alvo_foto:.3f}m) | "
            f"Erro XY: {err_xy_mm:5.1f}mm | Yaw: {tel['err_yaw_deg']:+5.1f}° -> {nome_arq}"
        )
        return True

    for idx_t, trecho in enumerate(trechos):
        descendo = trecho["descendo"]
        marcos_m = trecho["marcos_m"]
        idx_marco = 0
        inicio_na_borda = None

        print(
            f"\n[*] Trecho {idx_t + 1}/{len(trechos)} (Ciclo {trecho['ciclo']} - {trecho['dir_label']}): "
            f"{marcos_m[0]:.3f}m -> {marcos_m[-1]:.3f}m ({len(marcos_m)} fotos)..."
        )

        while idx_marco < len(marcos_m) and len(registros) < total_fotos:
            agora = time.time()
            dt = clamp(agora - t_ant, 0.005, 0.15)
            t_ant = agora

            # Mantém trim_alt congelado durante a oscilação para evitar windup e overshoot
            tel = controlador.atualizar_e_comandar(alt_cmd, usar_trim_xy=True, usar_trim_alt=False)
            alt_real = tel["alt_real"]
            pausado = tel["dist_xy"] > tol_xy_pausa_m

            if not pausado:
                if descendo:
                    if alt_real <= alt_min_m + 0.0025:
                        # Guarda de borda inferior: nunca comanda abaixo do limite mínimo
                        alt_cmd = max(alt_cmd, piso_cmd)
                    else:
                        dist_fim = max(0.0, alt_real - piso_cmd)
                        vel_local = vel_captura_ms * clamp(dist_fim / 0.025, 0.30, 1.0)
                        piso_dinamico = max(piso_cmd, alt_real - 0.012)
                        alt_cmd = clamp(max(piso_dinamico, alt_cmd - vel_local * dt), piso_cmd, teto_cmd)
                else:
                    if alt_real >= alt_max_m - 0.0025:
                        # Guarda de borda superior: nunca comanda acima do limite máximo
                        alt_cmd = min(alt_cmd, teto_cmd)
                    else:
                        dist_fim = max(0.0, teto_cmd - alt_real)
                        vel_local = vel_captura_ms * clamp(dist_fim / 0.025, 0.30, 1.0)
                        teto_dinamico = min(teto_cmd, alt_real + 0.012)
                        alt_cmd = clamp(min(teto_dinamico, alt_cmd + vel_local * dt), piso_cmd, teto_cmd)

            frame, _, _ = node_camera.obter_frame()
            tecla = exibir_hud_e_ler_tecla(
                mostrar_janela,
                nome_janela,
                frame,
                f"ETAPA 3: C{trecho['ciclo']}/{ciclos} {trecho['dir_label']} [{alt_min_m:.2f}-{alt_max_m:.2f}m]",
                tel,
                alt_cmd,
                fotos_salvas=len(registros),
                total_fotos=total_fotos,
                pausado=pausado,
            )
            if tecla in (ord("q"), ord("Q"), 27):
                raise KeyboardInterrupt

            alt_alvo_foto = marcos_m[idx_marco]
            atingiu_marco = (
                (alt_real <= alt_alvo_foto + 0.0008)
                if descendo
                else (alt_real >= alt_alvo_foto - 0.0008)
            )

            # Se estabilizou junto da borda de reversão sem cruzar os últimos ~2mm, não trava o trecho
            perto_borda = (
                (alt_real <= alt_min_m + 0.005)
                if descendo
                else (alt_real >= alt_max_m - 0.005)
            )
            if perto_borda and not pausado:
                if inicio_na_borda is None:
                    inicio_na_borda = agora
            else:
                inicio_na_borda = None

            timeout_borda = (inicio_na_borda is not None) and ((agora - inicio_na_borda) >= 0.8)

            if not pausado and (atingiu_marco or timeout_borda):
                if salvar_foto(trecho, alt_alvo_foto, tel):
                    idx_marco += 1

            time.sleep(0.025)

    return registros


def salvar_csv_registros(pasta_saida, registros):
    """Salva o log científico das fotos capturadas em CSV."""
    if not registros:
        return None
    ts_arq = datetime.now().strftime("%Y%m%d_%H%M%S")
    caminho_csv = os.path.join(pasta_saida, f"registro_fotos_{ts_arq}.csv")
    colunas = list(registros[0].keys())
    with open(caminho_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=colunas)
        writer.writeheader()
        writer.writerows(registros)
    return caminho_csv


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Voa até exatamente acima da armadilha a 0.50m, desce até 0.35m e "
            "captura 200 fotos indo e voltando entre 0.35m e 0.20m sem ultrapassar os limites."
        )
    )
    parser.add_argument("--conexao", default=DEFAULT_CONEXAO, help="Porta MAVLink (padrão: udp:127.0.0.1:14550)")
    parser.add_argument("--world", default=DEFAULT_WORLD_GAZEBO, help="Nome do mundo Gazebo (padrão: iris_runway)")
    parser.add_argument("--alt-transito", type=float, default=ALT_TRANSITO_PADRAO, help="Altitude de deslocamento horizontal seguro até a armadilha (padrão: 1.00 m)")
    parser.add_argument("--alt-max", type=float, default=ALT_FAIXA_MAX, help="Altitude máxima da varredura de fotos (padrão: 0.35 m)")
    parser.add_argument("--alt-min", type=float, default=ALT_FAIXA_MIN, help="Altitude mínima da varredura de fotos (padrão: 0.20 m)")
    parser.add_argument("--total-fotos", type=int, default=DEFAULT_TOTAL_FOTOS, help="Total de fotos a capturar (padrão: 200)")
    parser.add_argument("--ciclos", type=int, default=DEFAULT_CICLOS, help="Número de ciclos completos de descida+subida (padrão: 5)")
    parser.add_argument("--vel-preparo", type=float, default=DEFAULT_VEL_DESCIDA_PREPARO, help="Velocidade de descida entre 0.50m e 0.35m em m/s (padrão: 0.04 m/s)")
    parser.add_argument("--vel-captura", type=float, default=DEFAULT_VEL_VARREDURA_Z, help="Velocidade vertical na varredura 0.35m<->0.20m em m/s (padrão: 0.025 m/s)")
    parser.add_argument("--alt-retorno", type=float, default=ALT_INICIAL_ALVO, help="Altitude segura de retorno após concluir as fotos (padrão: 0.50 m)")
    parser.add_argument("--tol-xy", type=float, default=DEFAULT_TOL_XY_M, help="Tolerância horizontal para estabilização em m (padrão: 0.012 m)")
    parser.add_argument("--tol-xy-pausa", type=float, default=DEFAULT_TOL_XY_DESCIDA_M, help="Erro XY máximo durante varredura antes de pausar para recentralizar (padrão: 0.020 m)")
    parser.add_argument("--gimbal-yaw", type=float, default=DEFAULT_GIMBAL_YAW, help="Yaw do gimbal em graus (padrão: -90°, igual à missão)")
    parser.add_argument("--manter-yaw", action="store_true", help="Mantém a proa inicial do drone sem alinhar ao Yaw da armadilha")
    parser.add_argument("--offset-norte", type=float, default=0.0, help="Offset adicional em Norte (m) em relação ao centro da armadilha")
    parser.add_argument("--offset-leste", type=float, default=0.0, help="Offset adicional em Leste (m) em relação ao centro da armadilha")
    parser.add_argument("--saida", default=DEFAULT_PASTA_SAIDA, help="Pasta onde as fotos e o CSV serão salvos")
    parser.add_argument("--sem-gui", action="store_true", help="Executa sem abrir janela OpenCV")
    args = parser.parse_args()

    if args.alt_max <= args.alt_min:
        print("\n[ERRO] --alt-max deve ser maior que --alt-min.\n")
        sys.exit(1)

    if rclpy is None:
        print("\n[ERRO] ROS 2 (rclpy) não encontrado. Rode: source /opt/ros/jazzy/setup.bash\n")
        sys.exit(1)

    if not vehicle_module_ok:
        print("\n[ERRO] DroneKit ou pymavlink não estão instalados!\n")
        sys.exit(1)

    from controle_voo.gimbal import apontar_gimbal_nadir
    from controle_voo.movimento import configurar_parametro, enviar_velocidade
    from controle_voo.takeoff import arm_and_takeoff

    sem_display = not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY")
    mostrar_janela = not (args.sem_gui or sem_display)

    pasta_saida = os.path.abspath(args.saida)
    os.makedirs(pasta_saida, exist_ok=True)
    contador_inicial = obter_proximo_numero_captura(pasta_saida)

    print("=" * 75)
    print(f" CAPTURA EM VARREDURA VERTICAL ({args.alt_max:.2f}m <-> {args.alt_min:.2f}m) - {args.total_fotos} FOTOS")
    print("=" * 75)
    print(f" Conexão MAVLink  : {args.conexao}")
    print(f" Mundo Gazebo     : {args.world}")
    print(f" Faixa Vertical   : [{args.alt_min:.2f} m, {args.alt_max:.2f} m] ({args.ciclos} ciclos ida/volta)")
    print(f" Meta de Fotos    : {args.total_fotos} imagens (inicia em captura_{contador_inicial:03d}_...)")
    print(f" Gimbal Yaw       : {args.gimbal_yaw:+.1f}° (Nadir -90°)")
    print(f" Tolerância XY    : <= {args.tol_xy*1000:.1f} mm (pausa > {args.tol_xy_pausa*1000:.1f} mm)")
    print(f" Pasta de Saída   : {pasta_saida}")
    print(f" Interface OpenCV : {'ATIVA' if mostrar_janela else 'HEADLESS'}")
    print("=" * 75)

    rclpy.init()
    node_camera = ReceptorCameraROS(DEFAULT_TOPICO_CAMERA)
    executor_ros = ExecutorROSBG(node_camera)
    executor_ros.iniciar()

    monitor_gz = MonitorPoseGazebo(world_name=args.world)
    gz_iniciado = monitor_gz.iniciar()

    print(f"\n[*] Conectando ao drone em {args.conexao}...")
    try:
        vehicle = connect(args.conexao, wait_ready=False, heartbeat_timeout=15)
        if not aguardar_atributos(vehicle):
            print("[!] ERRO: Telemetria do drone não respondeu.")
            sys.exit(1)
    except Exception as exc:
        print(f"[!] ERRO AO CONECTAR: {exc}")
        sys.exit(1)

    print("[+] Drone conectado!")

    # Aguarda o stream de câmera antes de decolar para garantir que a bridge ROS está ativa
    print(f"[*] Aguardando primeiro frame da câmera ({DEFAULT_TOPICO_CAMERA})...")
    t_cam = time.time()
    while time.time() - t_cam < 20.0:
        frame, _, _ = node_camera.obter_frame()
        if frame is not None:
            print("[+] Feed da câmera conectado!")
            break
        time.sleep(0.1)
    else:
        print("[!] ERRO: Nenhuma imagem recebida em /camera/image.")
        print("    Verifique se a ponte ROS2 está rodando:")
        print("    ros2 run ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image@gz.msgs.Image")
        monitor_gz.parar()
        executor_ros.parar()
        node_camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        vehicle.close()
        sys.exit(1)

    z_solo_drone = DEFAULT_Z_SOLO_DRONE
    if gz_iniciado:
        print("[*] Aguardando leitura das poses do Gazebo...")
        poses_ini = monitor_gz.aguardar_poses(timeout=5.0)
        if poses_ini is not None:
            pos_d_ini, _, pos_a_ini, _ = poses_ini
            alt_ap_ini = vehicle.location.global_relative_frame.alt or 0.0
            if (not vehicle.armed) or alt_ap_ini < 0.15:
                z_solo_drone = pos_d_ini["z"]
            print(
                f"[+] Gazebo sincronizado! z_solo_drone={z_solo_drone:.4f}m | "
                f"Armadilha=({pos_a_ini['x']:.3f}, {pos_a_ini['y']:.3f}, {pos_a_ini['z']:.3f})m"
            )
        else:
            print("[!] Aviso: tópico /world/.../pose/info não respondeu; usando fallback LOCAL_NED.")

    nome_janela = "Camera VANT - Varredura 0.35m <-> 0.20m sobre Armadilha"
    if mostrar_janela:
        cv2.namedWindow(nome_janela, cv2.WINDOW_AUTOSIZE)

    registros = []

    try:
        configurar_parametro(vehicle, "WP_YAW_BEHAVIOR", 0)
        apontar_gimbal_nadir(vehicle, yaw_deg=args.gimbal_yaw)

        alt_atual = vehicle.location.global_relative_frame.alt or 0.0
        if (not vehicle.armed) or alt_atual < 0.25:
            alt_decolagem = max(ALT_INICIAL_ALVO, args.alt_transito)
            print(f"\n[*] Decolando para {alt_decolagem:.2f}m de altitude de trânsito...")
            arm_and_takeoff(vehicle, alt_decolagem)
        elif vehicle.mode.name != "GUIDED":
            vehicle.mode = VehicleMode("GUIDED")
            while vehicle.mode.name != "GUIDED":
                time.sleep(0.2)

        controlador = ControladorPrecisaoArmadilha(
            vehicle=vehicle,
            monitor_gz=monitor_gz,
            z_solo_drone=z_solo_drone,
            manter_yaw=args.manter_yaw,
            offset_norte=args.offset_norte,
            offset_leste=args.offset_leste,
        )

        # Etapa 1: Voar até exatamente acima da armadilha a 0.50 m
        print(f"\n[*] ETAPA 1: Voando até exatamente acima da armadilha a {ALT_INICIAL_ALVO:.2f}m...")
        executar_etapa_alinhamento(
            controlador=controlador,
            node_camera=node_camera,
            mostrar_janela=mostrar_janela,
            nome_janela=nome_janela,
            alvo_alt_m=ALT_INICIAL_ALVO,
            tol_xy_m=args.tol_xy,
            tol_alt_m=DEFAULT_TOL_ALT_M,
            tol_yaw_deg=DEFAULT_TOL_YAW_DEG,
            tempo_sustentado_s=1.2,
            timeout_s=45.0,
            titulo="ETAPA 1: ALINHANDO @ 0.50m ACIMA DA ARMADILHA",
            alt_transito_m=max(ALT_INICIAL_ALVO, args.alt_transito),
        )

        # Etapa 2: Estando exatamente acima dela, descer de 0.50 m até ~0.346 m (garantindo <= 0.35 m)
        inset_entrada = min(0.004, (args.alt_max - args.alt_min) * 0.03)
        descer_para_altitude_alinhado(
            controlador=controlador,
            node_camera=node_camera,
            mostrar_janela=mostrar_janela,
            nome_janela=nome_janela,
            alt_inicial_m=ALT_INICIAL_ALVO,
            alt_alvo_m=args.alt_max - inset_entrada,
            vel_descida_ms=args.vel_preparo,
            tol_xy_m=args.tol_xy,
            tol_xy_pausa_m=args.tol_xy_pausa,
            tol_alt_m=0.003,
            tol_yaw_deg=DEFAULT_TOL_YAW_DEG,
        )

        # Etapa 3: 200 fotos indo e voltando entre 0.35 m e 0.20 m sem ultrapassar os limites
        registros = executar_varredura_vertical_com_fotos(
            controlador=controlador,
            node_camera=node_camera,
            mostrar_janela=mostrar_janela,
            nome_janela=nome_janela,
            pasta_saida=pasta_saida,
            contador_inicial=contador_inicial,
            alt_max_m=args.alt_max,
            alt_min_m=args.alt_min,
            total_fotos=args.total_fotos,
            ciclos=args.ciclos,
            vel_captura_ms=args.vel_captura,
            tol_xy_pausa_m=args.tol_xy_pausa,
        )

        # Etapa 4: Retorno seguro para altitude acima da armadilha (0.50 m) mantendo posição
        print(f"\n[*] ETAPA 4: Retornando com segurança para {args.alt_retorno:.2f}m acima da armadilha...")
        executar_etapa_alinhamento(
            controlador=controlador,
            node_camera=node_camera,
            mostrar_janela=mostrar_janela,
            nome_janela=nome_janela,
            alvo_alt_m=args.alt_retorno,
            tol_xy_m=args.tol_xy_pausa,
            tol_alt_m=0.025,
            tol_yaw_deg=5.0,
            tempo_sustentado_s=0.6,
            timeout_s=15.0,
            titulo=f"ETAPA 4: RETORNO SEGURO @ {args.alt_retorno:.2f}m",
            alt_transito_m=args.alt_retorno,
        )

        caminho_csv = salvar_csv_registros(pasta_saida, registros)

        print("\n" + "=" * 75)
        print(" MISSÃO DE CAPTURA CONCLUÍDA COM SUCESSO!")
        print("=" * 75)
        print(f" Total de fotos salvas : {len(registros)}")
        if registros:
            erros_mm = [r["erro_xy_mm"] for r in registros]
            alts_m = [r["alt_gazebo_m"] for r in registros]
            print(
                f" Faixa de Altitude Real: {min(alts_m):.4f} m até {max(alts_m):.4f} m "
                f"(Limites: [{args.alt_min:.2f}, {args.alt_max:.2f}] m)"
            )
            print(
                f" Erro XY (mm)          : médio={sum(erros_mm)/len(erros_mm):.2f}mm | "
                f"mín={min(erros_mm):.2f}mm | máx={max(erros_mm):.2f}mm"
            )
        print(f" Pasta das fotos       : {pasta_saida}")
        if caminho_csv:
            print(f" Log CSV detalhado     : {caminho_csv}")
        print(f" Status de voo         : Pairado com segurança a {args.alt_retorno:.2f}m (GUIDED)")
        print("=" * 75 + "\n")

    except KeyboardInterrupt:
        print("\n[!] Operação interrompida pelo usuário.")
        if registros:
            caminho_csv = salvar_csv_registros(pasta_saida, registros)
            if caminho_csv:
                print(f"[i] Log parcial salvo em: {caminho_csv}")
        try:
            enviar_velocidade(vehicle, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
    finally:
        try:
            if vehicle.armed and (vehicle.location.global_relative_frame.alt or 0.0) > 0.10:
                enviar_velocidade(vehicle, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass

        if mostrar_janela:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        monitor_gz.parar()
        executor_ros.parar()
        node_camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        vehicle.close()
        print("[+] Conexões encerradas.")


if __name__ == "__main__":
    main()
