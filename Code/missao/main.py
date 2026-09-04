# main.py
# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

from dronekit import connect
import time
import rclpy
import cv2

from controle_voo.gimbal import apontar_gimbal_nadir
from controle_voo.takeoff import arm_and_takeoff
from controle_voo.movimento import ir_para_posicao_local, enviar_velocidade
from visao.rastreador import RastreadorYOLO

CONEXAO = 'udp:127.0.0.1:14550'

def main():
    print(f"Conectando ao drone em: {CONEXAO}")
    vehicle = connect(CONEXAO, wait_ready=True)
    rclpy.init()

    try:
        print("[+] Conexão bem-sucedida! Iniciando missão...")

        # 1. Aponta câmera para baixo
        print("[+] Estabilizando o gimbal em nadir...")
        apontar_gimbal_nadir(vehicle)
        
        # 2. Decolagem (Decolando mais alto: 6 metros para compensar a gaiola)
        altitude_inicial = 6.0
        arm_and_takeoff(vehicle, altitude_inicial)
        
        # 3. Navegação GPS Local (O Ajuste do SDF!)
        # Vai para X=3m (Frente), Y=0m, Z=-6m (Mantém os 6m de altura, lembrando que NED o Z é negativo para cima)
        print("[+] Navegando para as coordenadas aproximadas da armadilha (X=3.0)...")
        ir_para_posicao_local(vehicle, frente_x=3.0, direita_y=0.0, baixo_z=-6.0)
        
        # Dá 5 segundos para o drone voar esses 3 metros fisicamente no Gazebo
        time.sleep(5.0) 
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