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
    """Mantém a câmera em -90 graus e com yaw travado no referencial da Terra."""
    pitch_deg = _limitar(-90.0, PITCH_MIN_DEG, PITCH_MAX_DEG)
    yaw_deg = _limitar(yaw_deg, YAW_MIN_DEG, YAW_MAX_DEG)
    
    yaw_lock = mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK

    mensagem = vehicle.mav.command_long_encode(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        0,
        pitch_deg,
        yaw_deg,
        math.nan,
        math.nan,
        yaw_lock,
        0,
        0,
    )
    
    # CORREÇÃO: Envia a mensagem utilizando a interface .mav
    vehicle.mav.send(mensagem)
    
    if hasattr(vehicle, 'flush'):
        vehicle.flush()

    # O mount é atualizado a 50 Hz; esta pausa evita armar antes do primeiro
    # setpoint chegar aos controladores do Gazebo.
    time.sleep(0.25)
    print(f"[SUCESSO] Gimbal travado em Pitch: {pitch_deg}°, Yaw: {yaw_deg}°")

if __name__ == '__main__':
    # Conecta ao SITL do ArduPilot via UDP (padrão é 14550)
    print("Aguardando conexão com o drone...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    
    # Aguarda o primeiro heartbeat para garantir que a conexão foi estabelecida
    master.wait_heartbeat()
    print("Conexão estabelecida!")

    # Chama a função para apontar a câmera
    apontar_gimbal_nadir(master)