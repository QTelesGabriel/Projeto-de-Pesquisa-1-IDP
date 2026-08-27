# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

from dronekit import connect
from gimbal import apontar_gimbal_nadir
from takeoff import arm_and_takeoff
import time

# String de conexão padrão para ArduPilot SITL rodando na mesma máquina
CONEXAO = 'udp:127.0.0.1:14550'

def main():
    print(f"Conectando ao drone em: {CONEXAO}")
    # wait_ready=True garante que o script só avance quando todos os parâmetros forem baixados
    vehicle = connect(CONEXAO, wait_ready=True)

    try:
        print("Conexão bem-sucedida! Iniciando missão...")

        print("Apontando e estabilizando o gimbal em nadir...")
        apontar_gimbal_nadir(vehicle)
        
        # Chama a função importada para decolar a 5 metros
        altitude_desejada = 5.0
        arm_and_takeoff(vehicle, altitude_desejada)
        
        print("Mantendo o hover (voo pairado) por 10 segundos...")
        time.sleep(10)
        
        print("Missão concluída.")
        
    except KeyboardInterrupt:
        print("Missão interrompida pelo usuário.")
        
    finally:
        print("Fechando conexão com o drone...")
        vehicle.close()

if __name__ == "__main__":
    main()
