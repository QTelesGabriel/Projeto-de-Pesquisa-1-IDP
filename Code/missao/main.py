# main.py
# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

from dronekit import connect, VehicleMode
import sys
import time
import rclpy
import cv2
import random

# Importações dos módulos customizados
from controle_voo.takeoff import arm_and_takeoff
from controle_voo.gimbal import apontar_gimbal_nadir
from controle_voo.movimento import ir_para_posicao_local, enviar_velocidade, girar_drone
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
        # Vai para a região aproximada da gaiola (Gazebo X=50, Y=0) com um erro aleatório
        # IMPORTANTE: Gazebo X = Leste, Gazebo Y = Norte. 
        # Ardupilot LOCAL_NED: X = Norte (frente_x), Y = Leste (direita_y)
        alvo_gazebo_x = 50.0 + random.uniform(-5.0, 5.0)
        alvo_gazebo_y = 0.0 + random.uniform(-5.0, 5.0)
        
        # Mapeamento ENU (Gazebo) -> NED (Ardupilot)
        frente_norte = alvo_gazebo_y
        direita_leste = alvo_gazebo_x
        
        print(f"[+] Navegando para as coordenadas aproximadas da armadilha (Gazebo X={alvo_gazebo_x:.2f}, Y={alvo_gazebo_y:.2f})...")
        ir_para_posicao_local(vehicle, frente_x=frente_norte, direita_y=direita_leste, baixo_z=-altitude_inicial)
        
        # Dá 15 segundos para o drone voar esses ~50 metros fisicamente no Gazebo
        time.sleep(15.0) 
        
        # Rotaciona para provar que o Yaw visual da Fase 2 funciona
        angulo_aleatorio = random.uniform(0, 360)
        print(f"[+] Posicionamento concluído. Bagunçando o Yaw do drone para {angulo_aleatorio:.1f} graus...")
        girar_drone(vehicle, angulo_aleatorio)
        
        tempo_espera = 20
        print(f"[+] Aguardando {tempo_espera} segundos para estabilização da rotação...")
        for i in range(tempo_espera, 0, -1):
            sys.stdout.write(f"\rIniciando Fase 1 (Visual) em: {i} segundos... ")
            sys.stdout.flush()
            time.sleep(1.0)
        print() # Quebra de linha após o contador
        
        print("[+] Voo estabilizado! Iniciando Inteligência Visual...")
        
        # 4. Inicia o Cérebro ROS (Visão Computacional)
        rastreador_node = RastreadorYOLO(vehicle)
        
        try:
            # O ROS assume o controle dos motores a partir daqui
            rclpy.spin(rastreador_node)
        except SystemExit:
            print("\n[+] Fase Macro concluída! O Drone desceu a 2m. Iniciando Fase Micro...")
            # Limpa os processos da fase 1
            rastreador_node.destroy_node()
            cv2.destroyAllWindows()
            
            # 5. Inicia o Cérebro ROS da Fase 2 (Ajuste Fino)
            from visao.rastreador_fino import RastreadorFinoYOLO
            rastreador_fino_node = RastreadorFinoYOLO(vehicle)
            
            try:
                rclpy.spin(rastreador_fino_node)
            except SystemExit:
                print("\n[+] Fase Micro concluída! Alvo capturado. Iniciando retorno (Fase 3)...")
                rastreador_fino_node.destroy_node()
                cv2.destroyAllWindows()
                
                # 6. Fase 3 - Inspeção (Pouso sobre o alvo)
                from dronekit import VehicleMode
                print("[+] Acionando modo LAND (Pouso)...")
                vehicle.mode = VehicleMode("LAND")
                
                print("[+] Missão Finalizada com sucesso! O VANT irá pousar e desligar os motores sobre a armadilha.")
                print("[+] Você poderá inspecionar visualmente o alinhamento 3D no Gazebo.")
                print("[+] Aguardando o desarme automático...")
                
                # Manter o script rodando até que o VANT pouse e desarme
                while vehicle.armed:
                    time.sleep(1)
                    
                print("\n[+] VANT desarmado sobre a armadilha! Simulação encerrada com sucesso.")
            
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
