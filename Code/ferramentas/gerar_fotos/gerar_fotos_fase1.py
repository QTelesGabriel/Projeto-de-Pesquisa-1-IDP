# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

import sys
import os
import time
from dronekit import connect, APIException

caminho_mission = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../mission'))
sys.path.append(caminho_mission)

from takeoff import arm_and_takeoff
from gimbal import apontar_gimbal_nadir
from scanner import escanear_gaiola_grid

CONEXAO = 'udp:127.0.0.1:14550'

def aguardar_atributos(vehicle, timeout=30):
    start = time.time()
    print("[*] Aguardando leitura dos dados basicos (mode, location, armable)...")
    while time.time() - start < timeout:
        if vehicle.mode.name and vehicle.location.global_relative_frame and vehicle.is_armable is not None:
            print("[+] Dados recebidos com sucesso!")
            return True
        time.sleep(1)
        print(f"   ... esperando. mode={vehicle.mode.name}, loc={vehicle.location.global_relative_frame}, armable={vehicle.is_armable}")
    return False

def main():
    print(f"[*] Conectando ao drone em: {CONEXAO}...")
    try:
        # wait_ready=False evita o bug do DroneKit de travar na leitura completa de parametros
        vehicle = connect(CONEXAO, wait_ready=False, heartbeat_timeout=15)
        print("[*] Conexão UDP e Heartbeat estabelecidos!")
        
        # Espera customizada apenas para os atributos que realmente importam
        if not aguardar_atributos(vehicle, timeout=40):
            print("\n[!] ERRO: O drone conectou, mas os dados de voo (GPS/IMU) nao estao chegando!")
            print(" -> Verifique a janela do Gazebo: o drone explodiu ou esta caindo no vazio?")
            print(" -> Verifique o sim_vehicle.py: O EKF do ArduPilot parou de mandar dados?")
            sys.exit(1)

    except Exception as e:
        print(f"\n[!] ERRO CRITICO AO CONECTAR: {e}")
        print(" -> Verifique se o sim_vehicle.py ja terminou de compilar/iniciar.")
        print(" -> Verifique se o QGroundControl nao esta aberto roubando a porta 14550.")
        sys.exit(1)

    try:
        print("\n[+] Missão de Fotogrametria pronta para iniciar!")

        print("\n[!] Apontando e estabilizando o gimbal em nadir...")
        apontar_gimbal_nadir(vehicle)
        
        ponto_origem = vehicle.location.global_relative_frame
        arm_and_takeoff(vehicle, 2.0)
        
        posicao_gaiola_norte = 0.0
        posicao_gaiola_leste = 3.0
        
        lista_alturas = [float(h) for h in range(2, 22, 2)]
        
        print("\nIniciando Loop Fotogramétrico (pressione Ctrl+C para pousar)...")
        
        while True:
            escanear_gaiola_grid(
                vehicle, 
                ponto_zero=ponto_origem, 
                alvo_norte=posicao_gaiola_norte, 
                alvo_leste=posicao_gaiola_leste, 
                alturas=lista_alturas
            )
            print("\n[!] Varredura completa! Reiniciando em 3 segundos...")
            time.sleep(3)

    except KeyboardInterrupt:
        print("\nMissão interrompida! Retornando e Pousando (RTL)...")
        vehicle.mode = "RTL"
        
        while vehicle.armed:
            print(f" Descendo... Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
            time.sleep(2)
            
        print("Pouso finalizado com sucesso!")
        
    finally:
        vehicle.close()

if __name__ == "__main__":
    main()
