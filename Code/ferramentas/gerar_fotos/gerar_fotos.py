# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

import sys
import os
import time
from dronekit import connect

# Ensinando o Python a voltar DUAS pastas para achar a 'mission'
caminho_mission = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../mission'))
sys.path.append(caminho_mission)

from takeoff import arm_and_takeoff
from scanner import escanear_gaiola

CONEXAO = 'udp:127.0.0.1:14550'

def main():
    print(f"Conectando ao drone em: {CONEXAO}")
    vehicle = connect(CONEXAO, wait_ready=True)

    try:
        print("Conexão bem-sucedida! Preparando para gerar dataset...")
        
        ponto_origem = vehicle.location.global_relative_frame
        
        # Inicia decolando na primeira altura do teste (2.0)
        arm_and_takeoff(vehicle, 2.0)
        
        posicao_gaiola_norte = 0.0
        posicao_gaiola_leste = 3.0
        
        # Geração dinâmica da lista de alturas: de 2 a 20 metros (pulando de 2 em 2)
        # O Python vai gerar exatamente: [2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]
        lista_alturas = [float(h) for h in range(2, 22, 2)]
        
        print("\nIniciando o loop de movimentação (pressione Ctrl+C no terminal para parar e pousar)...")
        
        # LOOP INFINITO
        while True:
            escanear_gaiola(
                vehicle, 
                ponto_zero=ponto_origem, 
                alvo_norte=posicao_gaiola_norte, 
                alvo_leste=posicao_gaiola_leste, 
                alturas=lista_alturas,
                pontos_por_circulo=8
            )
            print("\n[!] Varredura completa de 2m a 20m! Reiniciando ciclo em 5 segundos...")
            time.sleep(5)

    except KeyboardInterrupt:
        print("\nLoop interrompido pelo usuário! Modo RTL ativado (Voltando pra casa)...")
        vehicle.mode = "RTL"
        
        while vehicle.armed:
            print(f" Descendo... Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
            time.sleep(2)
            
        print("Pouso finalizado com sucesso!")
        
    finally:
        vehicle.close()

if __name__ == "__main__":
    main()