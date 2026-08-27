# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

import sys
import os
import time
from dronekit import connect

caminho_mission = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../mission'))
sys.path.append(caminho_mission)

from takeoff import arm_and_takeoff
from gimbal import apontar_gimbal_nadir
from scanner import escanear_gaiola_grid

CONEXAO = 'udp:127.0.0.1:14550'

def main():
    print(f"Conectando ao drone em: {CONEXAO}")
    vehicle = connect(CONEXAO, wait_ready=True)

    try:
        print("Conexão bem-sucedida! Preparando missão de Fotogrametria...")

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
