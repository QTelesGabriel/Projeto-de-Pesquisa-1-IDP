# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

import sys
import os
from dronekit import connect

# Importa a função de movimento exato (LOCAL NED - Sem erro de curvatura de GPS)
caminho_mission = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../missao'))
sys.path.append(caminho_mission)

from controle_voo.movimento import ir_para_posicao_local
from controle_voo.takeoff import arm_and_takeoff

CONEXAO = 'udp:127.0.0.1:14550'

import time

def aguardar_atributos(vehicle, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.mode.name and vehicle.location.global_relative_frame and vehicle.is_armable is not None:
            return True
        time.sleep(1)
    return False

def main():
    print(f"[*] Conectando ao drone em {CONEXAO}...")
    vehicle = connect(CONEXAO, wait_ready=False)
    
    print("[*] Aguardando leitura da telemetria...")
    if not aguardar_atributos(vehicle):
        print("[!] Timeout na telemetria. Feche o QGroundControl se estiver aberto.")
        sys.exit(1)
        
    print("[+] Conectado e Telemetria sincronizada!")
    
    if vehicle.location.global_relative_frame.alt < 0.5:
        print("[!] Drone no chão. Decolando para 2.0 metros...")
        arm_and_takeoff(vehicle, 2.0)
    else:
        print(f"[+] Drone já está no ar a {vehicle.location.global_relative_frame.alt:.2f}m")

    print("\n" + "="*50)
    print("🚁 CONTROLE MANUAL ABSOLUTO (LOCAL_NED)")
    print("="*50)
    print("Referencial a partir do ponto de NASCIMENTO no Gazebo:")
    print(" - Eixo X: Para frente (Norte do Gazebo)")
    print(" - Eixo Y: Para a direita (Leste do Gazebo)")
    print(" - Eixo Z: Para CIMA (Valores NEGATIVOS. Ex: -2.0 sobe 2 metros)")
    print("Exemplo de comando: 0.0 50.0 -2.0")
    print("Digite 'sair' para encerrar o programa.\n")

    while True:
        comando = input("Digite X Y Z (ou 'sair'): ")
        
        if comando.lower() in ['sair', 'exit', 'quit']:
            break
            
        partes = comando.split()
        if len(partes) != 3:
            print("[!] Erro: Você deve digitar exatamente 3 números separados por espaço.")
            continue
            
        try:
            x = float(partes[0])
            y = float(partes[1])
            z = float(partes[2])
            
            print(f" -> Movendo para Frente(X)={x:.2f}m, Direita(Y)={y:.2f}m, Altura(Z)={-z:.2f}m")
            # Envia o comando MAVLink LOCAL_NED (Precisão absoluta plana, ignora a curvatura da Terra)
            ir_para_posicao_local(vehicle, frente_x=x, direita_y=y, baixo_z=z)
            
        except ValueError:
            print("[!] Erro: Apenas números são aceitos.")
            
    vehicle.close()
    print("Desconectado!")

if __name__ == "__main__":
    main()
