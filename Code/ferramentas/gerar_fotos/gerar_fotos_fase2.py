# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

import sys
import os
import time
import math
from dronekit import connect, LocationGlobalRelative
from pymavlink import mavutil

# Importa os módulos básicos do projeto
caminho_mission = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../missao'))
sys.path.append(caminho_mission)

from controle_voo.takeoff import arm_and_takeoff
from controle_voo.gimbal import apontar_gimbal_nadir

CONEXAO = 'udp:127.0.0.1:14550'

def deslocar_coordenada(local_original, dNorte, dLeste):
    """ Desloca uma coordenada GPS em X metros para o Norte e Y metros para o Leste """
    raio_terra = 6378137.0 
    dLat = dNorte / raio_terra
    dLon = dLeste / (raio_terra * math.cos(math.pi * local_original.lat / 180.0))
    nova_lat = local_original.lat + (dLat * 180.0 / math.pi)
    nova_lon = local_original.lon + (dLon * 180.0 / math.pi)
    return LocationGlobalRelative(nova_lat, nova_lon, local_original.alt)

def fixar_yaw_norte(vehicle):
    """Trava o bico do drone para o Norte."""
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        0, 0, 1, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def aguardar_atributos(vehicle, timeout=30):
    start = time.time()
    print("[*] Aguardando leitura dos dados basicos (mode, location, armable)...")
    while time.time() - start < timeout:
        if vehicle.mode.name and vehicle.location.global_relative_frame and vehicle.is_armable is not None:
            return True
        time.sleep(1)
    return False

def main():
    print(f"[*] Conectando ao drone em: {CONEXAO}...")
    try:
        vehicle = connect(CONEXAO, wait_ready=False, heartbeat_timeout=15)
        if not aguardar_atributos(vehicle, timeout=40):
            print("\n[!] ERRO: Falha ao ler os dados de telemetria do drone.")
            sys.exit(1)
    except Exception as e:
        print(f"\n[!] ERRO CRITICO AO CONECTAR: {e}")
        sys.exit(1)

    try:
        print("\n[+] Drone conectado com sucesso!")
        print("\n[!] Estabilizando o gimbal em nadir (90 graus p/ baixo)...")
        apontar_gimbal_nadir(vehicle)
        
        # Posição inicial: Como você já alinhou 100% via terminal, 
        # a origem da nossa espiral será o exato milímetro que o drone está no ar agora!
        ponto_origem = vehicle.location.global_relative_frame
        
        # Garante que ele comece a espiral de 2.0 metros (caso esteja em outra altura)
        if abs(ponto_origem.alt - 2.0) > 0.2:
            print(f"\n[!] Ajustando altura inicial de {ponto_origem.alt:.2f}m para 2.0m...")
            ponto_inicio = LocationGlobalRelative(ponto_origem.lat, ponto_origem.lon, 2.0)
            vehicle.simple_goto(ponto_inicio)
            time.sleep(4)
            ponto_origem = vehicle.location.global_relative_frame # Atualiza com a altura nova
            
        print("\n[+] Drone no ponto de início perfeito. Iniciando Loop Infinito da Espiral...")
        print("[DICA] Pressione Ctrl+C a qualquer momento para parar e pousar!\n")
        
        # --- PARÂMETROS DA ESPIRAL ---
        raio = 0.15 # 15 cm de raio
        altura_max = 2.0 # metros
        altura_min = 0.2 # metros (20cm do chão)
        
        rotacoes_por_viagem = 4 # Dará 4 voltas durante o trajeto
        passo_angular = 15 # Graus por waypoint
        
        # Mantém o bico sempre pro Norte enquanto orbita
        fixar_yaw_norte(vehicle)
        
        direcao = "DESCENDO"
        
        # Loop Infinito (Sobe e Desce)
        while True:
            angulos = range(0, rotacoes_por_viagem * 360, passo_angular)
            total_pontos = len(angulos)
            
            for idx, angulo_deg in enumerate(angulos):
                # 1. Interpolação Linear Z (Alternando entre Subir e Descer)
                progresso = idx / float(total_pontos - 1)
                
                if direcao == "DESCENDO":
                    alt_atual = altura_max - (progresso * (altura_max - altura_min))
                else:
                    alt_atual = altura_min + (progresso * (altura_max - altura_min))
                
                # 2. Equações Paramétricas do Círculo (A espiral gira ao redor do próprio eixo atual: 0.0)
                angulo_rad = math.radians(angulo_deg)
                dn = 0.0 + (raio * math.cos(angulo_rad))
                dl = 0.0 + (raio * math.sin(angulo_rad))
                
                # 3. Mapeia e Envia ao Drone
                waypoint = deslocar_coordenada(ponto_origem, dn, dl)
                waypoint.alt = alt_atual
                
                print(f" -> [{direcao}] Espiral: {idx+1}/{total_pontos} | Altura: {alt_atual:.2f}m | Angulo: {angulo_deg}°")
                vehicle.simple_goto(waypoint)
                
                # Pausa de meio segundo a cada 15 graus.
                time.sleep(0.5)
                
            # Inverte o estado da direção ao fim da viagem
            if direcao == "DESCENDO":
                direcao = "SUBINDO"
            else:
                direcao = "DESCENDO"
                
            print("\n[!] Invertendo sentido da viagem! Aguardando 2 segundos para estabilizar a inércia...")
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nMissão interrompida pelo usuário!")
        
    finally:
        print("\n[!] Subindo para 2m de volta e retornando ao ponto de início (RTL)...")
        vehicle.mode = "RTL"
        while vehicle.armed:
            time.sleep(1)
        vehicle.close()
        print("Finalizado!")

if __name__ == "__main__":
    main()
