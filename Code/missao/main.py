# main.py
# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

from dronekit import connect
import sys
import time
import rclpy
import cv2
import random

from controle_voo.gimbal import apontar_gimbal_nadir
from controle_voo.takeoff import arm_and_takeoff
from controle_voo.movimento import ir_para_posicao_local, enviar_velocidade
from visao.rastreador import RastreadorYOLO

CONEXAO = 'udp:127.0.0.1:14550'

def aguardar_atributos(vehicle, timeout=30):
    start = time.time()
    print("[*] Aguardando leitura dos dados basicos (mode, location, armable)...")
    while time.time() - start < timeout:
        if vehicle.mode.name and vehicle.location.global_relative_frame and vehicle.is_armable is not None:
            print("[+] Dados recebidos com sucesso!")
            return True
        time.sleep(1)
    return False

def main():
    print(f"Conectando ao drone em: {CONEXAO}")
    try:
        vehicle = connect(CONEXAO, wait_ready=False, heartbeat_timeout=15)
        print("[*] Conexão UDP e Heartbeat estabelecidos!")
        
        if not aguardar_atributos(vehicle, timeout=40):
            print("\n[!] ERRO: O drone conectou, mas os dados de voo (GPS/IMU) nao estao chegando!")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n[!] ERRO CRITICO AO CONECTAR: {e}")
        sys.exit(1)
        
    rclpy.init()

    try:
        print("[+] Conexão bem-sucedida! Iniciando missão...")

        # 1. Aponta câmera para baixo
        print("[+] Estabilizando o gimbal em nadir...")
        apontar_gimbal_nadir(vehicle)
        
        # 2. Decolagem
        altitude_inicial = 20.0
        arm_and_takeoff(vehicle, altitude_inicial)
        
        # 3. Navegação GPS Local Aproximada
        # Vai para a região aproximada da gaiola (x=50, y=0) com um erro aleatório (quadrado de lado 10m)
        alvo_x = 50.0 + random.uniform(-5.0, 5.0)
        alvo_y = 0.0 + random.uniform(-5.0, 5.0)
        print(f"[+] Navegando para as coordenadas aproximadas da armadilha (X={alvo_x:.2f}, Y={alvo_y:.2f})...")
        ir_para_posicao_local(vehicle, frente_x=alvo_x, direita_y=alvo_y, baixo_z=-altitude_inicial)
        
        # Dá 15 segundos para o drone voar esses ~50 metros fisicamente no Gazebo
        time.sleep(15.0) 
        print("[+] Posicionamento inicial concluído. Iniciando Inteligência Visual...")
        
        # 4. Inicia o Cérebro ROS (Visão Computacional)
        rastreador_node = RastreadorYOLO(vehicle)
        
        try:
            # O ROS assume o controle dos motores a partir daqui
            rclpy.spin(rastreador_node)
        except SystemExit:
            # Note que no rastreador.py precisamos ajustar aquele `if altitude <= 2.0:` para 3.0m
            print("\n[+] Fase Macro concluída! O Drone desceu e estabilizou perfeitamente sobre o alvo.")
            
        print("[+] Missão Finalizada. Mantendo Hover. Pressione Ctrl+C para encerrar...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[!] Missão interrompida pelo usuário.")
        
    finally:
        print("[i] Desligando motores e limpando processos...")
        try:
            enviar_velocidade(vehicle, 0.0, 0.0, 0.0)
        except:
            pass
            
        cv2.destroyAllWindows()
        if 'rastreador_node' in locals():
            rastreador_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        vehicle.close()

if __name__ == "__main__":
    main()
