"""Comandos para o gimbal SIYI A8 Mini simulado."""

import math
import time

from pymavlink import mavutil


PITCH_MIN_DEG = -135.0
PITCH_MAX_DEG = 45.0
YAW_MIN_DEG = -160.0
YAW_MAX_DEG = 160.0


def _limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, float(valor)))


def apontar_gimbal_nadir(vehicle, yaw_deg=0.0):
    """Mantem a camera em -90 graus, mas o Yaw acompanha o nariz do drone."""
    pitch_deg = _limitar(-90.0, PITCH_MIN_DEG, PITCH_MAX_DEG)
    # Sem compensação: a câmera deve olhar perfeitamente para frente (0 graus relativo ao corpo)
    yaw_deg = _limitar(yaw_deg, YAW_MIN_DEG, YAW_MAX_DEG)
    # GIMBAL_MANAGER_FLAGS_ROLL_LOCK (4) + GIMBAL_MANAGER_FLAGS_PITCH_LOCK (8) = 12
    # Isso trava o Pitch e o Roll no horizonte (estabilização), mas deixa o Yaw solto para acompanhar o drone
    flags = 12

    mensagem = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        0,
        pitch_deg,
        yaw_deg,
        math.nan,
        math.nan,
        flags,
        0,
        0,
    )
    vehicle.send_mavlink(mensagem)
    vehicle.flush()

    # O mount e atualizado a 50 Hz; esta pausa evita armar antes do primeiro
    # setpoint chegar aos controladores do Gazebo.
    time.sleep(0.25)

