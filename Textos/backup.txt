import math
import time
from dronekit import LocationGlobalRelative
from pymavlink import mavutil

def deslocar_coordenada(local_original, dNorte, dLeste):
    raio_terra = 6378137.0 
    dLat = dNorte / raio_terra
    dLon = dLeste / (raio_terra * math.cos(math.pi * local_original.lat / 180.0))
    nova_lat = local_original.lat + (dLat * 180.0 / math.pi)
    nova_lon = local_original.lon + (dLon * 180.0 / math.pi)
    return LocationGlobalRelative(nova_lat, nova_lon, local_original.alt)

def get_distancia_horizontal(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlong = loc2.lon - loc1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def fixar_yaw_norte(vehicle):
    """Trava o bico do drone para o Norte. Ele desliza sobre o grid como um caranguejo."""
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        0, 0, 1, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def escanear_gaiola_grid(vehicle, ponto_zero, alvo_norte, alvo_leste, alturas):
    """Cria um grid 3x3 acima do alvo. O tamanho se adapta à altitude e ao FOV da câmera."""
    print("\n--- INICIANDO MAPEAMENTO EM GRADE (RETANGULAR) ---")
    
    fixar_yaw_norte(vehicle)
    
    # Coordenadas do Grid: (Norte, Leste)
    # Linha 1 (vai), Linha 2 (volta), Linha 3 (vai)
    padrao_ziguezague = [
        (1, -1),  (1, 0),  (1, 1),
        (0, 1),   (0, 0),  (0, -1),
        (-1, -1), (-1, 0), (-1, 1)
    ]

    for alt in alturas:
        # A matemática da Siyi A8 Mini: A gaiola fica na tela se afastarmos 
        # até 35% da altura atual do drone em relação ao centro.
        offset_seguro = max(1.0, alt * 0.35)
        
        print(f"\n[!] Subindo para Altitude {alt}m | Borda da grade ajustada p/ {offset_seguro:.2f}m")
        
        for idx, (mult_norte, mult_leste) in enumerate(padrao_ziguezague):
            # Calcula o deslocamento exato
            dn = alvo_norte + (mult_norte * offset_seguro)
            dl = alvo_leste + (mult_leste * offset_seguro)
            
            waypoint = deslocar_coordenada(ponto_zero, dn, dl)
            waypoint.alt = alt
            
            print(f" -> Ponto {idx+1}/9 do Grid (Norte: {mult_norte}, Leste: {mult_leste})")
            vehicle.simple_goto(waypoint)
            
            inicio_tempo = time.time()
            
            # Loop de tolerância inteligente
            while True:
                pos_atual = vehicle.location.global_relative_frame
                dist = get_distancia_horizontal(pos_atual, waypoint)
                alt_diff = abs(pos_atual.alt - alt)
                
                if dist < 1.0 and alt_diff < 0.5:
                    print("    [+] Estabilizado! (Pausando para capturas limpas...)")
                    time.sleep(1.5) 
                    break
                
                if time.time() - inicio_tempo > 15:
                    print("    [!] Timeout atingido. Avançando...")
                    break
                
                time.sleep(0.5)