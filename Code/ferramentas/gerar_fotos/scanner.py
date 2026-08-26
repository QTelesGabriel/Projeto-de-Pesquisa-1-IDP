import math
import time
from dronekit import LocationGlobalRelative
from pymavlink import mavutil

def deslocar_coordenada(local_original, dNorte, dLeste):
    """Calcula nova coordenada GPS baseada em um deslocamento (Norte e Leste)."""
    raio_terra = 6378137.0 
    
    dLat = dNorte / raio_terra
    dLon = dLeste / (raio_terra * math.cos(math.pi * local_original.lat / 180.0))
    
    nova_lat = local_original.lat + (dLat * 180.0 / math.pi)
    nova_lon = local_original.lon + (dLon * 180.0 / math.pi)
    
    return LocationGlobalRelative(nova_lat, nova_lon, local_original.alt)

def get_distancia_horizontal(loc1, loc2):
    """Retorna a distância horizontal em metros."""
    dlat = loc2.lat - loc1.lat
    dlong = loc2.lon - loc1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def apontar_drone_para_alvo(vehicle, lat, lon, alt):
    """
    Usa o comando MAVLink SET_ROI para travar o Yaw do drone. 
    O drone sempre apontará o nariz para esta coordenada, mesmo voando de lado.
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION, 
        0,       
        0, 0, 0, 0, 
        lat, lon, alt
    )
    vehicle.send_mavlink(msg)

def calcular_raio_ideal(altura):
    """
    Calcula o raio proporcional à abertura (FOV) da câmera SIYI A8 Mini.
    FOV Horizontal = 1.4137 rad. Resolucao 16:9 (1080p).
    FOV Vertical calculado = aprox. 0.96 rad.
    
    A distância do centro da imagem até a borda vertical no chão é:
    borda = altura * tan(0.96 / 2) = altura * 0.48
    
    Queremos que a gaiola fique a 65% dessa distância (não no centro e não na borda).
    """
    borda_maxima = altura * 0.48
    raio_calculado = borda_maxima * 0.65
    
    # Adicionamos um raio mínimo de segurança de 1.5m para baixas altitudes
    return max(1.5, raio_calculado)

def escanear_gaiola(vehicle, ponto_zero, alvo_norte, alvo_leste, alturas, pontos_por_circulo=8):
    """Faz o drone voar em círculos otimizados para o FOV da câmera."""
    print("\n--- INICIANDO ROTINA DE ESCANEAMENTO FOTOGRÁFICO ---")
    
    # 1. Travar a "cabeça" do drone para olhar para a gaiola (ROI)
    loc_gaiola = deslocar_coordenada(ponto_zero, alvo_norte, alvo_leste)
    apontar_drone_para_alvo(vehicle, loc_gaiola.lat, loc_gaiola.lon, 0)
    print("    [+] Foco (ROI) travado na gaiola!")
    
    # 2. Varrer as alturas
    for alt in alturas:
        raio = calcular_raio_ideal(alt)
        print(f"\n[!] Subindo para Altitude {alt}m | Raio da órbita ajustado p/ {raio:.2f}m")
        
        for i in range(pontos_por_circulo):
            angulo_graus = (360.0 / pontos_por_circulo) * i
            angulo_rad = math.radians(angulo_graus)
            
            offset_norte = alvo_norte + (raio * math.cos(angulo_rad))
            offset_leste = alvo_leste + (raio * math.sin(angulo_rad))
            
            waypoint = deslocar_coordenada(ponto_zero, offset_norte, offset_leste)
            waypoint.alt = alt
            
            print(f" -> Navegando para o ponto {i+1}/{pontos_por_circulo} (Ângulo: {angulo_graus}º)")
            vehicle.simple_goto(waypoint)
            
            inicio_tempo = time.time()
            
            # Loop de espera com tolerância elástica
            while True:
                pos_atual = vehicle.location.global_relative_frame
                dist = get_distancia_horizontal(pos_atual, waypoint)
                alt_diff = abs(pos_atual.alt - alt)
                
                if dist < 1.5 and alt_diff < 1.0:
                    print("    [+] Ponto alcançado! (Tirando foto...)")
                    time.sleep(1) 
                    break
                
                # Timeout se o vento/inércia não deixarem cravar o ponto perfeitamente
                if time.time() - inicio_tempo > 15:
                    print("    [!] Ponto aproximado alcançado (Timeout). Tirando foto e avançando...")
                    time.sleep(1)
                    break
                
                time.sleep(0.5)