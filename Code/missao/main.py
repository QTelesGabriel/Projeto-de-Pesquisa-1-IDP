# main.py
# --- PARA O DRONEKIT FUNCIONAR NO PYTHON 3.10+ -----------
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
# ---------------------------------------------------------

from dronekit import connect, VehicleMode
import argparse
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
from gazebo_pose import parsear_pose_info_gz, ler_poses_absolutas_gazebo
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


def parsear_argumentos(argv=None):
    """Lê as opções de linha de comando para execução com/sem interface gráfica e salvamento automático."""
    parser = argparse.ArgumentParser(description="Missão autônoma de captura visual VTS")
    parser.add_argument(
        '--sem-gui',
        action='store_true',
        help='Executa sem abrir janelas OpenCV (ativado automaticamente se DISPLAY/WAYLAND_DISPLAY estiverem ausentes).',
    )
    parser.add_argument(
        '--auto',
        action='store_true',
        help='Registra automaticamente no CSV usando os critérios objetivos de T3, T4 e T5 sem pedir [S/N/D].',
    )
    parser.add_argument(
        '--teste-oclusao',
        action='store_true',
        help='Executa ensaio isolado do Teste 2 (EKF + oclusão simulada no Gazebo), sem gravar no Monte Carlo (T3/T4/T5).',
    )
    parser.add_argument(
        '--duracao-oclusao',
        type=float,
        default=3.8,
        help='Duração da oclusão total simulada na Etapa 3 em segundos (padrão: 3.8s, dentro do limite de 4.0s).',
    )
    return parser.parse_args(argv)


def main():
    args = parsear_argumentos()
    sem_display = not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY')
    mostrar_janela = not (args.sem_gui or sem_display)
    if args.teste_oclusao:
        print(f"[i] Modo interface OpenCV: {'ATIVA' if mostrar_janela else 'DESATIVADA (headless)'} | MODO ISOLADO: TESTE 2 EKF + OCLUSAO ({args.duracao_oclusao:.2f}s) [sem impacto no Teste 5]")
    else:
        print(f"[i] Modo interface OpenCV: {'ATIVA' if mostrar_janela else 'DESATIVADA (headless)'} | Registro CSV: {'AUTOMATICO (--auto)' if args.auto else 'CONFIRMACAO MANUAL [S/N/D]'}")
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
    rastreador_node = None
    rastreador_fino_node = None

    try:
        print("[+] Conexão bem-sucedida! Iniciando missão...")
        
        # Trava total de Yaw no ArduPilot: impede qualquer rotação automática na direção de voo
        print("[+] Configurando ArduPilot: trava absoluta de Yaw (WP_YAW_BEHAVIOR = 0)...")
        configurar_parametro(vehicle, 'WP_YAW_BEHAVIOR', 0)
        configurar_parametro(vehicle, 'LAND_SPD_MS', 0.8)

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
        
        # Inicializa logger isolado (Teste 2 no simulador) OU logger padrão (Testes 3/4/5)
        telemetria = None
        teste2_logger = None
        if args.teste_oclusao:
            from teste2_oclusao_logger import Teste2OclusaoLogger
            teste2_logger = Teste2OclusaoLogger(condicoes_iniciais={
                'alt_inicial_m': altitude_inicial,
                'erro_gps_m': raio_erro,
                'yaw_inicial_deg': angulo_aleatorio,
                'duracao_oclusao_cfg_s': args.duracao_oclusao,
            })
            print(f"[+] [TESTE 2 ISOLADO] Gravando dados exclusivos em: {teste2_logger.caminho_csv}")
        else:
            from telemetria_logger import TelemetriaLogger
            telemetria = TelemetriaLogger()
            print(f"[+] Gravando telemetria contínua em: {telemetria.nome_arquivo}")
        
        # 4. Inicia o Cérebro ROS (Visão Computacional)
        rastreador_node = RastreadorYOLO(
            vehicle,
            logger=telemetria,
            mostrar_janela=mostrar_janela,
            teste2_logger=teste2_logger,
        )
        
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
            rastreador_node = None
            if mostrar_janela:
                cv2.destroyAllWindows()
            
            # 5. Inicia o Cérebro ROS da Fase 2 (Ajuste Fino)
            from visao.rastreador_fino import RastreadorFinoYOLO
            rastreador_fino_node = RastreadorFinoYOLO(
                vehicle,
                logger=telemetria,
                mostrar_janela=mostrar_janela,
                teste_oclusao=args.teste_oclusao,
                duracao_oclusao=args.duracao_oclusao,
                teste2_logger=teste2_logger,
            )
            
            try:
                rclpy.spin(rastreador_fino_node)
            except SystemExit:
                tempo_total_missao = round(time.time() - tempo_inicio_missao, 2)
                print("\n[+] Fase Micro concluída! Alvo capturado. Pousando para inspeção visual...")
                
                # Extrai as métricas visuais registradas antes do Blind Drop (referência de voo)
                err_x_vis_mm = round((rastreador_fino_node.erro_final_x or 0.0) * 1000.0, 2)
                err_y_vis_mm = round((rastreador_fino_node.erro_final_y or 0.0) * 1000.0, 2)
                rmse_vis_mm = round(math.sqrt(err_x_vis_mm**2 + err_y_vis_mm**2), 2)
                err_yaw_deg = round(rastreador_fino_node.ultimo_erro_yaw_deg if rastreador_fino_node.ultimo_erro_yaw_deg is not None else 0.0, 2)
                gz_pre_resultado = rastreador_fino_node.gz_pre_resultado
                diag_thread = rastreador_fino_node.diagnostico_thread
                marcos_threads = list(rastreador_fino_node._threads_marcos)
                
                rastreador_fino_node.destroy_node()
                rastreador_fino_node = None
                if mostrar_janela:
                    cv2.destroyAllWindows()
                
                # 6. Fase 3 - Pouso e Desarme sobre a Gaiola
                from dronekit import VehicleMode
                print("[+] Acionando modo LAND (Pouso)...")
                vehicle.mode = VehicleMode("LAND")
                
                print("[+] Aguardando o toque no solo e o desarme automático dos motores...")
                while vehicle.armed:
                    time.sleep(0.25)
                    
                print("\n[+] VANT desarmado sobre a armadilha!")
                time.sleep(0.5)
                if diag_thread is not None:
                    diag_thread.join(timeout=2.0)
                for th in marcos_threads:
                    th.join(timeout=2.0)

                # Medição absoluta pós-pouso completo no mundo Gazebo (Ground Truth)
                poses_mundo = ler_poses_absolutas_gazebo(world_name='iris_runway')

                if args.teste_oclusao:
                    teste2_logger.registrar_marco('pos_pouso', poses_mundo, extra={'tempo_total_s': tempo_total_missao})
                    teste2_logger.fechar()
                    testes_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'Testes'))
                    if testes_dir not in sys.path:
                        sys.path.insert(0, testes_dir)
                    from teste_2_simulador_analise import analisar_voo_teste2_simulador
                    analisar_voo_teste2_simulador(teste2_logger.caminho_csv, teste2_logger.caminho_marcos)
                    return
                if poses_mundo is not None:
                    pos_drone = poses_mundo['drone_coaxial']['position']
                    pos_armadilha = poses_mundo['armadilha_alvo']['position']
                    err_x_mm = round((pos_drone['x'] - pos_armadilha['x']) * 1000.0, 2)
                    err_y_mm = round((pos_drone['y'] - pos_armadilha['y']) * 1000.0, 2)
                    rmse_mm = round(math.sqrt(err_x_mm**2 + err_y_mm**2), 2)
                    fonte_erro_t5 = "MUNDO GAZEBO (Pós-Pouso Completo)"
                else:
                    print("[!] Aviso: falha ao ler /world/iris_runway/pose/info; usando erro visual pré-Blind Drop.")
                    pos_drone = None
                    pos_armadilha = None
                    err_x_mm, err_y_mm, rmse_mm = err_x_vis_mm, err_y_vis_mm, rmse_vis_mm
                    fonte_erro_t5 = "VISAO PRE-BLIND DROP (Fallback)"
                
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
                print(f" [TESTE 5 - ACOPLAMENTO & OFFSET ({fonte_erro_t5})]:")
                if pos_drone is not None and pos_armadilha is not None:
                    print(f"   -> Pos. Drone Mundo : X={pos_drone['x']:.4f} m | Y={pos_drone['y']:.4f} m | Z={pos_drone['z']:.4f} m")
                    print(f"   -> Pos. Alvo Mundo  : X={pos_armadilha['x']:.4f} m | Y={pos_armadilha['y']:.4f} m | Z={pos_armadilha['z']:.4f} m")
                print(f"   -> Erro Mundo X (L) : {err_x_mm:+.2f} mm")
                print(f"   -> Erro Mundo Y (N) : {err_y_mm:+.2f} mm")
                print(f"   -> Erro Radial Mundo: {rmse_mm:.2f} mm (Meta TCC: <= 8.0 mm)")
                print(f"   -> (Ref. Visão pré-Blind Drop : {rmse_vis_mm:.2f} mm | X={err_x_vis_mm:+.2f}, Y={err_y_vis_mm:+.2f})")
                if gz_pre_resultado:
                    print(f"   -> (Ref. Gazebo pré-Blind Drop: {gz_pre_resultado['rmse_mm']:.2f} mm | X={gz_pre_resultado['dx_mm']:+.2f}, Y={gz_pre_resultado['dy_mm']:+.2f} | Diff={gz_pre_resultado['diff_mm']:.2f} mm)")
                else:
                    print("   -> (Ref. Gazebo pré-Blind Drop: indisponível)")
                print("-" * 68)
                # Fechamento do logger de telemetria
                telemetria.fechar()
                
                # Avaliação analítica de critérios do TCC
                criterio_t3 = (t3_fov_retido == 'SIM')
                criterio_t4 = (abs(err_yaw_deg) <= 1.0)
                criterio_t5 = (rmse_mm <= 8.0)
                sucesso_automatico = (criterio_t3 and criterio_t4 and criterio_t5)
                status_sugerido = 'SIM' if sucesso_automatico else 'NAO'
                
                print("🔍 Avaliação Analítica dos Critérios do TCC:")
                print(f"   -> [T3] 100% Retido no FOV  : {'ATENDIDO' if criterio_t3 else 'NAO ATENDIDO'}")
                print(f"   -> [T4] Erro Yaw <= 1.0°    : {'ATENDIDO' if criterio_t4 else 'NAO ATENDIDO'} ({err_yaw_deg:+.2f}°)")
                print(f"   -> [T5] RMSE <= 8.0 mm      : {'ATENDIDO' if criterio_t5 else 'NAO ATENDIDO'} ({rmse_mm:.2f} mm)")
                print(f"   => Classificação Automática: {status_sugerido}")
                print("-" * 68)
                if args.auto:
                    status_str = status_sugerido
                    print(f"[+] Registro automático (--auto): voo classificado como {status_str} pelos critérios T3/T4/T5.")
                else:
                    print("  [S] Confirmar SUCESSO | [N] Marcar FALHA | [D] DESCARTAR")
                    
                    status_str = None
                    while True:
                        try:
                            escolha = input(f">> Confirme o resultado do voo [S/N/D] (Sugestão automática: {status_sugerido}): ").strip().upper()
                        except EOFError:
                            escolha = 'D'
                        
                        if escolha in ['S', 'SIM']:
                            status_str = 'SIM'
                            break
                        elif escolha in ['N', 'NAO', 'NÃO']:
                            status_str = 'NAO'
                            break
                        elif escolha in ['D', 'DESCARTAR']:
                            status_str = None
                            break
                        else:
                            print("[!] Opção inválida. Digite S (Sucesso), N (Falha) ou D (Descartar).")
                    
                caminho_csv = os.path.join(os.path.dirname(__file__), 'relatorio_testes_monte_carlo.csv')
                colunas_csv = [
                    'Timestamp', 'Alt_Inicial_m', 'Erro_GPS_Raio_m', 'Yaw_Inicial_deg', 'Tempo_Total_s',
                    'T3_FOV_Retido', 'T3_Max_Afastamento_pct', 'T3_Bloqueios_Histerese',
                    'T4_Erro_Yaw_Final_deg',
                    'T5_Erro_X_mm', 'T5_Erro_Y_mm', 'T5_RMSE_Final_mm', 'T5_Acoplamento_Confirmado',
                    'Arquivo_Telemetria',
                    'T5_Erro_X_PreBlindDrop_mm', 'T5_Erro_Y_PreBlindDrop_mm', 'T5_RMSE_PreBlindDrop_mm'
                ]
                
                if status_str is not None:
                    arquivo_existe = os.path.exists(caminho_csv) and os.path.getsize(caminho_csv) > 0
                    if arquivo_existe:
                        with open(caminho_csv, 'r', newline='', encoding='utf-8') as f_in:
                            linhas_existentes = list(csv.reader(f_in))
                        if not linhas_existentes:
                            arquivo_existe = False
                        elif len(linhas_existentes[0]) < len(colunas_csv):
                            linhas_migradas = [colunas_csv]
                            for linha_antiga in linhas_existentes[1:]:
                                faltantes = len(colunas_csv) - len(linha_antiga)
                                linhas_migradas.append(linha_antiga + [''] * max(0, faltantes))
                            with open(caminho_csv, 'w', newline='', encoding='utf-8') as f_out:
                                csv.writer(f_out).writerows(linhas_migradas)
                    with open(caminho_csv, 'a', newline='', encoding='utf-8') as f_csv:
                        writer = csv.writer(f_csv)
                        if not arquivo_existe:
                            writer.writerow(colunas_csv)
                        writer.writerow([
                            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                            altitude_inicial, raio_erro, angulo_aleatorio, tempo_total_missao,
                            t3_fov_retido, t3_max_afastamento, t3_bloqueios,
                            err_yaw_deg,
                            err_x_mm, err_y_mm, rmse_mm, status_str,
                            telemetria.nome_arquivo,
                            err_x_vis_mm, err_y_vis_mm, rmse_vis_mm
                        ])
                    print(f"\n[+] Voo registrado com sucesso em: {os.path.basename(caminho_csv)} ({status_str})")
                    print(f"[+] Telemetria gravada em: {telemetria.nome_arquivo}")
                    
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

                            rmses_pre = [float(l[16]) for l in linhas if len(l) > 16 and l[16] != '']
                            rmse_pre_medio = sum(rmses_pre) / len(rmses_pre) if rmses_pre else None
                            
                            yaws = [abs(float(l[8])) for l in linhas if len(l) > 8]
                            yaw_medio = sum(yaws) / len(yaws) if yaws else 0.0
                            
                            print("-" * 68)
                            print(f"📊 HISTÓRICO CONSOLIDADO ({total_testes} missões oficiais gravadas):")
                            print(f"   [Teste 3] Taxa de Retenção FOV   : {fov_100_pct}/{total_testes} ({taxa_fov:.1f}%) [Meta: 100%]")
                            print(f"   [Teste 4] Erro Médio de Yaw      : {yaw_medio:.2f}° [Meta: <= 1.0°]")
                            if rmse_pre_medio is not None:
                                print(f"   [Teste 5] Erro Pré-Blind Drop    : {rmse_pre_medio:.2f} mm")
                            print(f"   [Teste 5] Erro Pós-LAND (Mundo)  : {rmse_medio:.2f} mm (Mín: {min(rmses):.2f} mm, Máx: {max(rmses):.2f} mm)")
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
            
        if 'mostrar_janela' in locals() and mostrar_janela:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        if 'telemetria' in locals() and telemetria is not None:
            try:
                telemetria.fechar()
            except:
                pass
        if 'teste2_logger' in locals() and teste2_logger is not None:
            try:
                teste2_logger.fechar()
            except:
                pass
        if rastreador_node is not None:
            rastreador_node.destroy_node()
        if rastreador_fino_node is not None:
            rastreador_fino_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        vehicle.close()

if __name__ == "__main__":
    main()
