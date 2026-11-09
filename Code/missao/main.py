# main.py
# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

from dronekit import connect, VehicleMode
import sys
import os
import csv
import math
import time
import rclpy
import cv2
import random
from datetime import datetime

# Importações dos módulos customizados
from controle_voo.takeoff import arm_and_takeoff
from controle_voo.gimbal import apontar_gimbal_nadir
from controle_voo.movimento import ir_para_posicao_local, enviar_velocidade, girar_drone, configurar_parametro
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
        
        # Trava total de Yaw no ArduPilot: impede qualquer rotação automática na direção de voo
        print("[+] Configurando ArduPilot: trava absoluta de Yaw (WP_YAW_BEHAVIOR = 0)...")
        configurar_parametro(vehicle, 'WP_YAW_BEHAVIOR', 0)

        # 1. Aponta câmera para baixo e VIRA ELA 90 GRAUS PARA A ESQUERDA (-90)
        # Isso garante que desde a Fase 1 o drone vai voar com o chassi 90º rotacionado à direita
        print("[+] Estabilizando o gimbal em nadir (Rotacionado -90 graus)...")
        apontar_gimbal_nadir(vehicle, yaw_deg=-90.0)
        
        # =====================================================================
        # CONFIGURAÇÃO ESTOCÁSTICA REALISTA (TESTE 5 / MONTE CARLO DO TCC)
        # =====================================================================
        # 1. Altitude Inicial com tolerância barométrica realista (16.0m a 20.0m)
        altitude_inicial = round(random.uniform(16.0, 20.0), 2)
        
        # 2. Decolagem
        print(f"[+] Decolagem para altitude estocástica de cruzeiro: {altitude_inicial:.2f}m...")
        arm_and_takeoff(vehicle, altitude_inicial)
        
        # 3. Dispersão de erro do GPS civil (Raio radial entre 1.0m e 4.5m)
        # A 16m-20m, 4.5m garante 100% de presença dentro do FOV da câmera nadir
        raio_erro = round(random.uniform(1.0, 4.5), 2)
        angulo_erro = random.uniform(0.0, 2.0 * math.pi)
        disp_x = round(raio_erro * math.cos(angulo_erro), 2)
        disp_y = round(raio_erro * math.sin(angulo_erro), 2)
        alvo_gazebo_x = 50.0 + disp_x
        alvo_gazebo_y = 0.0 + disp_y
        
        # Mapeamento ENU (Gazebo) -> NED (Ardupilot)
        frente_norte = alvo_gazebo_y
        direita_leste = alvo_gazebo_x
        
        # 4. Orientação de proa (Yaw) inicial aleatória (0.0° a 360.0°)
        angulo_aleatorio = round(random.uniform(0.0, 360.0), 1)
        
        print(f"[+] Navegando para região aproximada do GPS (Erro radial={raio_erro:.2f}m, Alt={altitude_inicial:.2f}m)...")
        ir_para_posicao_local(vehicle, frente_x=frente_norte, direita_y=direita_leste, baixo_z=-altitude_inicial)
        
        # Monitora a aproximação dinâmica até o ponto GPS (evita esperas fixas desnecessárias)
        inicio_deslocamento = time.time()
        while time.time() - inicio_deslocamento < 25.0:
            loc = vehicle.location.local_frame
            if loc and loc.north is not None and loc.east is not None:
                dist_gps = math.sqrt((loc.north - frente_norte)**2 + (loc.east - direita_leste)**2)
                if dist_gps < 1.2:
                    print(f"[+] Waypoint GPS alcançado (distância residual: {dist_gps:.2f}m)!")
                    break
            time.sleep(0.5)
            
        # Rotaciona para o Yaw inicial estocástico
        print(f"[+] Posicionamento concluído. Rotacionando Yaw inicial aleatório: {angulo_aleatorio:.1f}°...")
        girar_drone(vehicle, angulo_aleatorio)
        time.sleep(5.0) # Estabilização da atitude angular
        
        print("\n[+] Voo estabilizado! Iniciando Inteligência Visual...")
        tempo_inicio_missao = time.time()
        
        # 4. Inicia o Cérebro ROS (Visão Computacional)
        rastreador_node = RastreadorYOLO(vehicle)
        
        try:
            # O ROS assume o controle dos motores a partir daqui
            rclpy.spin(rastreador_node)
        except SystemExit:
            print("\n[+] Fase Macro concluída! O Drone desceu a 2m. Iniciando Fase Micro...")
            # Extrai as métricas científicas do Teste 3 (Histerese e FOV) antes de destruir o nó
            t3_max_afastamento = round(rastreador_node.max_afastamento_fov_pct, 1)
            t3_bloqueios = rastreador_node.bloqueios_histerese
            t3_saiu_fov = rastreador_node.saiu_fisicamente_fov
            t3_fov_retido = "SIM" if (not t3_saiu_fov and t3_max_afastamento < 100.0) else "NAO"
            
            # Limpa os processos da fase 1
            rastreador_node.destroy_node()
            cv2.destroyAllWindows()
            
            # 5. Inicia o Cérebro ROS da Fase 2 (Ajuste Fino)
            from visao.rastreador_fino import RastreadorFinoYOLO
            rastreador_fino_node = RastreadorFinoYOLO(vehicle)
            
            try:
                rclpy.spin(rastreador_fino_node)
            except SystemExit:
                tempo_total_missao = round(time.time() - tempo_inicio_missao, 2)
                print("\n[+] Fase Micro concluída! Alvo capturado. Pousando para inspeção visual...")
                
                # Extrai as métricas finais registradas no nó antes de destruí-lo (Testes 4 e 5)
                err_x_mm = round((rastreador_fino_node.erro_final_x or 0.0) * 1000.0, 2)
                err_y_mm = round((rastreador_fino_node.erro_final_y or 0.0) * 1000.0, 2)
                rmse_mm = round(math.sqrt(err_x_mm**2 + err_y_mm**2), 2)
                err_yaw_deg = round(rastreador_fino_node.ultimo_erro_yaw_deg if rastreador_fino_node.ultimo_erro_yaw_deg is not None else 0.0, 2)
                
                rastreador_fino_node.destroy_node()
                cv2.destroyAllWindows()
                
                # 6. Fase 3 - Pouso e Desarme sobre a Gaiola
                from dronekit import VehicleMode
                print("[+] Acionando modo LAND (Pouso)...")
                vehicle.mode = VehicleMode("LAND")
                
                print("[+] Aguardando o toque no solo e o desarme automático dos motores...")
                while vehicle.armed:
                    time.sleep(1)
                    
                print("\n[+] VANT desarmado sobre a armadilha!")
                
                # =============================================================
                # CONSOLIDAÇÃO MULTI-TESTE (TESTES 3, 4 E 5 UNIFICADOS)
                # =============================================================
                print("\n" + "="*68)
                print("🎯 RESULTADO INTEGRADO DO VOO (TESTES 3, 4 E 5 DO TCC)")
                print("="*68)
                print(f" Condições Iniciais    : Alt={altitude_inicial:.2f}m | Erro GPS={raio_erro:.2f}m | Yaw={angulo_aleatorio:.1f}°")
                print(f" Tempo Total de Voo    : {tempo_total_missao:.1f} segundos")
                print("-" * 68)
                print(f" [TESTE 3 - FOV/HISTERESE]:")
                print(f"   -> Retenção na Tela : {'100% Retido (NUNCA saiu da imagem)' if t3_fov_retido == 'SIM' else 'ALVO ESCAPOU DO FOV'}")
                print(f"   -> Maior Afastamento: {t3_max_afastamento:.1f}% da borda (Bloqueios de descida: {t3_bloqueios})")
                print(f" [TESTE 4 - YAW]:")
                print(f"   -> Erro Final de Yaw: {err_yaw_deg:+.2f}° (Meta TCC: <= 1.0°)")
                print(f" [TESTE 5 - ACOPLAMENTO & OFFSET]:")
                print(f"   -> Erro Final em X  : {err_x_mm:+.2f} mm")
                print(f"   -> Erro Final em Y  : {err_y_mm:+.2f} mm")
                print(f"   -> RMSE Cravado     : {rmse_mm:.2f} mm (Meta TCC: <= 8.0 mm)")
                print("-" * 68)
                print("🔍 Olhe no Gazebo 3D para verificar a posição física da garra:")
                print("  [S] Salvar como SUCESSO (A garra acoplou perfeitamente no gancho)")
                print("  [N] Salvar como FALHA   (A garra errou o gancho ou deslocou a gaiola)")
                print("  [D] DESCARTAR / TESTE   (Não salvar no CSV, apenas teste experimental)")
                print("-" * 68)
                
                try:
                    escolha = input(">> Deseja salvar este voo? [S/N/D] (Padrão: D): ").strip().upper()
                except Exception:
                    escolha = 'D'
                    
                if escolha in ['S', 'SIM']:
                    status_str = 'SIM'
                elif escolha in ['N', 'NAO', 'NÃO']:
                    status_str = 'NAO'
                else:
                    status_str = None
                    
                caminho_csv = os.path.join(os.path.dirname(__file__), 'relatorio_testes_monte_carlo.csv')
                
                if status_str is not None:
                    arquivo_existe = os.path.exists(caminho_csv)
                    with open(caminho_csv, 'a', newline='', encoding='utf-8') as f_csv:
                        writer = csv.writer(f_csv)
                        if not arquivo_existe:
                            writer.writerow([
                                'Timestamp', 'Alt_Inicial_m', 'Erro_GPS_Raio_m', 'Yaw_Inicial_deg', 'Tempo_Total_s',
                                'T3_FOV_Retido', 'T3_Max_Afastamento_pct', 'T3_Bloqueios_Histerese',
                                'T4_Erro_Yaw_Final_deg',
                                'T5_Erro_X_mm', 'T5_Erro_Y_mm', 'T5_RMSE_Final_mm', 'T5_Acoplamento_Confirmado'
                            ])
                        writer.writerow([
                            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                            altitude_inicial, raio_erro, angulo_aleatorio, tempo_total_missao,
                            t3_fov_retido, t3_max_afastamento, t3_bloqueios,
                            err_yaw_deg,
                            err_x_mm, err_y_mm, rmse_mm, status_str
                        ])
                    print(f"\n[+] Voo registrado com sucesso em: {os.path.basename(caminho_csv)} ({status_str})")
                    
                    # Exibe histórico consolidado
                    try:
                        with open(caminho_csv, 'r', encoding='utf-8') as f_read:
                            linhas = list(csv.reader(f_read))[1:]
                            total_testes = len(linhas)
                            sucessos_t5 = sum(1 for l in linhas if len(l) > 12 and l[12] == 'SIM')
                            taxa_acoplamento = (sucessos_t5 / total_testes) * 100.0 if total_testes > 0 else 0.0
                            
                            fov_100_pct = sum(1 for l in linhas if len(l) > 5 and l[5] == 'SIM')
                            taxa_fov = (fov_100_pct / total_testes) * 100.0 if total_testes > 0 else 0.0
                            
                            rmses = [float(l[11]) for l in linhas if len(l) > 11]
                            rmse_medio = sum(rmses) / len(rmses) if rmses else 0.0
                            
                            yaws = [abs(float(l[8])) for l in linhas if len(l) > 8]
                            yaw_medio = sum(yaws) / len(yaws) if yaws else 0.0
                            
                            print("-" * 68)
                            print(f"📊 HISTÓRICO CONSOLIDADO ({total_testes} missões oficiais gravadas):")
                            print(f"   [Teste 3] Taxa de Retenção FOV   : {fov_100_pct}/{total_testes} ({taxa_fov:.1f}%) [Meta: 100%]")
                            print(f"   [Teste 4] Erro Médio de Yaw      : {yaw_medio:.2f}° [Meta: <= 1.0°]")
                            print(f"   [Teste 5] RMSE Médio de Posição  : {rmse_medio:.2f} mm (Mín: {min(rmses):.2f} mm, Máx: {max(rmses):.2f} mm)")
                            print(f"   [Teste 5] Taxa Real Acoplamento  : {sucessos_t5}/{total_testes} ({taxa_acoplamento:.1f}%) [Meta TCC: > 60%]")
                    except Exception:
                        pass
                else:
                    print("\n[i] VOO DESCARTADO: Nenhum dado foi adicionado ao arquivo CSV.")
                    print(f"[i] Para reiniciar o histórico a qualquer momento, basta apagar o arquivo:")
                    print(f"    rm {caminho_csv}")
                    
                print("="*68 + "\n")
            
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
