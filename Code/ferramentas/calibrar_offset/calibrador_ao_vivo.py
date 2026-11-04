#!/usr/bin/env python3
"""
=============================================================================
Calibrador de Offset Definitivo ao Vivo (0.40m)
=============================================================================
Este script ajuda a encontrar o offset definitivo da garra:
1. Conecta ao drone (DroneKit/MAVLink) e à câmera ROS 2 (/camera/image).
2. Estabiliza o gimbal em Nadir (-90° pitch, -90° yaw, idêntico à missão).
3. Move o drone até a posição exata da armadilha (Norte=0.0m, Leste=50.0m) a 0.40m.
4. Trava o Yaw de acordo com a orientação da missão.
5. Permite micro-ajustes finos no plano horizontal (W/A/S/D) para alinhar
   a garra fisicamente no Gazebo 3D.
6. Calcula em tempo real o offset exato entre a mira da câmera e a Bounding Box do YOLO.
7. Pressione ESPAÇO para salvar a foto e copiar os valores exatos para o rastreador_fino.py!
=============================================================================
"""

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

import sys
import os
import time
import math
import cv2
import numpy as np

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# DroneKit / MAVLink
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from ultralytics import YOLO

CAMINHO_MISSAO = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../missao'))
sys.path.append(CAMINHO_MISSAO)

from controle_voo.takeoff import arm_and_takeoff
from controle_voo.gimbal import apontar_gimbal_nadir
from controle_voo.movimento import enviar_velocidade, configurar_parametro

CONEXAO = 'udp:127.0.0.1:14550'
ALVO_NORTE_NED = 0.0
ALVO_LESTE_NED = 50.0
ALTURA_ALVO = 0.35  # 0.35m (altura exata de hover da Etapa 3B)

class CalibradorAoVivoNode(Node):
    def __init__(self, vehicle):
        super().__init__('calibrador_ao_vivo_offset')
        self.vehicle = vehicle
        self.bridge = CvBridge()
        
        # Modelo YOLO
        caminho_modelo = os.path.join(CAMINHO_MISSAO, 'visao', 'modelos_IA', 'YOLOv11n_Fase2_Armadilha_Alcas.pt')
        if not os.path.exists(caminho_modelo):
            caminho_modelo = os.path.join(os.path.dirname(__file__), '../../modelos_IA/YOLOv11n_Fase2_Armadilha_Alcas.pt')
        
        self.get_logger().info(f"Carregando modelo: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)
        
        self.cx = 960.0
        self.cy = 540.0
        self.fx = 1124.0
        self.fy = 1124.0
        self.ultimo_frame = None
        self.offset_calculado = None
        
        self.sub_cam = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        self.nome_janela = "Calibrador de Offset ao Vivo - Pressione ESPACO para Salvar"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.ultimo_frame = cv_image.copy()
        
        # Predição YOLO
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.45)
        boxes = resultados[0].boxes
        
        # Mira da Câmera (Centro Óptico)
        cv2.drawMarker(cv_image, (int(self.cx), int(self.cy)), (0, 255, 255), cv2.MARKER_CROSS, 40, 2)
        cv2.putText(cv_image, "MIRA DA CAMERA (Centro Optico)", (int(self.cx) + 15, int(self.cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Distância óptica real da câmera até a tampa da gaiola (z_camera = alt + 0.320m, z_alvo = 0.317m)
        alt_rel = max(0.005, (self.vehicle.location.global_relative_frame.alt + 0.320) - 0.317)

        boxes_armadilha = [b for b in boxes if int(b.cls[0].item()) == 1]
        
        if len(boxes_armadilha) > 0:
            best = max(boxes_armadilha, key=lambda b: float(b.conf[0].item()))
            x1, y1, x2, y2 = map(int, best.xyxy[0].cpu().numpy())
            w = x2 - x1
            h = y2 - y1
            box_cx = x1 + w / 2.0
            box_cy = y1 + h / 2.0
            
            # Desenha a Armadilha
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.circle(cv_image, (int(box_cx), int(box_cy)), 6, (255, 165, 0), -1)
            cv2.putText(cv_image, "Centro BBox Armadilha", (int(box_cx) + 10, int(box_cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 165, 0), 1)
            
            # Linha ligando o Centro da BBox até a Mira Óptica da Câmera
            cv2.line(cv_image, (int(box_cx), int(box_cy)), (int(self.cx), int(self.cy)), (0, 255, 0), 2)
            
            # CÁLCULO EXATO DO OFFSET VISUAL PROPORCIONAL
            # Corresponde exatamente à fórmula do rastreador_fino.py:
            # alvo_x = box_cx + (box_w * offset_pct_x)  => offset_pct_x = (cx - box_cx) / box_w
            # alvo_y = box_cy - (box_h * offset_pct_y)  => offset_pct_y = (box_cy - cy) / box_h
            offset_x = (self.cx - box_cx) / float(w)
            offset_y = (box_cy - self.cy) / float(h)
            self.offset_calculado = (offset_x, offset_y, w, h, box_cx, box_cy)
            
            # Distância física projetada no plano da tampa (em milímetros)
            dist_x_mm = (self.cx - box_cx) * alt_rel / self.fx * 1000.0
            dist_y_mm = (self.cy - box_cy) * alt_rel / self.fy * 1000.0
            
            # Painel Informativo
            overlay = cv_image.copy()
            cv2.rectangle(overlay, (20, 20), (740, 165), (20, 20, 20), -1)
            cv2.addWeighted(overlay, 0.7, cv_image, 0.3, 0, cv_image)
            
            cv2.putText(cv_image, f"CALIBRACAO DE OFFSET (@ Altura {self.vehicle.location.global_relative_frame.alt:.2f}m):", 
                        (35, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.putText(cv_image, f"Offset Calculado: self.offset_pct_x = {offset_x:+.4f} | self.offset_pct_y = {offset_y:+.4f}", 
                        (35, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(cv_image, f"Deslocamento no Solo: dX = {dist_x_mm:+.1f} mm | dY = {dist_y_mm:+.1f} mm", 
                        (35, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            cv2.putText(cv_image, f"BBox Tamanho: {w}x{h} px | Centro: ({box_cx:.1f}, {box_cy:.1f})", 
                        (35, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
            cv2.putText(cv_image, "Pressione [ESPACO] para salvar | [W/A/S/D] Micro-ajuste | [Q] Sair", 
                        (35, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
        else:
            cv2.putText(cv_image, "BUSCANDO ARMADILHA...", (30, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow(self.nome_janela, cv_image)
        tecla = cv2.waitKey(1) & 0xFF
        
        # Micro-ajustes via teclado no drone (2 cm por toque)
        pas = 0.02
        if tecla == ord('w') or tecla == ord('W'):
            enviar_velocidade(self.vehicle, pas, 0.0, 0.0, 0.0)
        elif tecla == ord('s') or tecla == ord('S'):
            enviar_velocidade(self.vehicle, -pas, 0.0, 0.0, 0.0)
        elif tecla == ord('a') or tecla == ord('A'):
            enviar_velocidade(self.vehicle, 0.0, -pas, 0.0, 0.0)
        elif tecla == ord('d') or tecla == ord('D'):
            enviar_velocidade(self.vehicle, 0.0, pas, 0.0, 0.0)
        elif tecla == ord(' '):
            if self.offset_calculado is not None:
                ox, oy, w, h, bcx, bcy = self.offset_calculado
                print("\n" + "="*60)
                print("🎯 OFFSET DEFINITIVO CAPTURADO COM SUCESSO!")
                print("="*60)
                print(f"Altitude Bruta do Drone : {self.vehicle.location.global_relative_frame.alt:.3f} m")
                print(f"Largura da BBox (w)     : {w} px")
                print(f"Altura da BBox  (h)     : {h} px")
                print(f"Centro BBox (cx, cy)    : ({bcx:.1f}, {bcy:.1f}) px")
                print(f"Mira Câmera (cx, cy)    : ({self.cx:.1f}, {self.cy:.1f}) px")
                print("-" * 60)
                print("COPIE E COLE NO rastreador_fino.py:")
                print(f"self.offset_pct_x = {ox:.4f}")
                print(f"self.offset_pct_y = {oy:.4f}")
                print("="*60 + "\n")
                
                # Salva foto para documentação do TCC
                pasta_fotos = os.path.join(os.path.dirname(__file__), 'fotos_offset')
                os.makedirs(pasta_fotos, exist_ok=True)
                nome_foto = os.path.join(pasta_fotos, f"calibracao_offset_{int(time.time())}.jpg")
                cv2.imwrite(nome_foto, self.ultimo_frame)
                print(f"[+] Foto de documentação salva em: {nome_foto}\n")
        elif tecla == ord('q') or tecla == ord('Q') or tecla == 27:
            rclpy.shutdown()

def ir_para_posicao_precisa(vehicle, alvo_norte, alvo_leste, alvo_alt, timeout=40.0):
    """Navega para a posição e paira estável."""
    print(f"[*] Navegando para Norte={alvo_norte:.2f}m, Leste={alvo_leste:.2f}m, Alt={alvo_alt:.2f}m...")
    inicio = time.time()
    
    while time.time() - inicio < timeout:
        alt_atual = vehicle.location.global_relative_frame.alt
        loc = vehicle.location.local_frame
        
        dn = (loc.north - alvo_norte) if loc and loc.north else 999.0
        dl = (loc.east - alvo_leste) if loc and loc.east else 999.0
        dist_h = math.sqrt(dn**2 + dl**2) if loc and loc.north else 999.0
        erro_alt = abs(alt_atual - alvo_alt)
        
        # Envia alvo LOCAL_NED com trava total de Yaw (ignora qualquer comando de rotação, sem virar o drone)
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, # Bits 10 e 11 em 1: ignora Yaw e Yaw Rate (não vira a proa do drone)
            alvo_norte, alvo_leste, -alvo_alt,
            0, 0, 0,
            0, 0, 0,
            0.0, 0.0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        
        sys.stdout.write(f"\r -> Dist H: {dist_h:.2f}m | Alt Real: {alt_atual:.2f}m (Alvo: {alvo_alt:.2f}m)...   ")
        sys.stdout.flush()
        
        if dist_h < 0.25 and erro_alt < 0.10:
            print("\n[+] Posicionamento concluído!")
            break
        time.sleep(0.2)

def aguardar_atributos(vehicle, timeout=30):
    start = time.time()
    print("[*] Aguardando leitura dos dados basicos (mode, location, armable)...")
    while time.time() - start < timeout:
        if vehicle.mode.name and vehicle.location.global_relative_frame and vehicle.location.global_relative_frame.alt is not None:
            print("[+] Dados de telemetria recebidos com sucesso!")
            return True
        time.sleep(1)
    return False

def main():
    print("="*60)
    print(" FERRAMENTA DE OBTENÇÃO DO OFFSET DEFINITIVO")
    print("="*60)
    print(f"[*] Conectando ao drone em {CONEXAO}...")
    vehicle = connect(CONEXAO, wait_ready=False, heartbeat_timeout=15)
    
    if not aguardar_atributos(vehicle):
        print("[!] Timeout aguardando telemetria do drone. Encerrando.")
        sys.exit(1)
        
    rclpy.init()
    
    print("[*] Configurando Gimbal em Nadir (Apenas Pitch -90°, SEM virar Yaw)...")
    apontar_gimbal_nadir(vehicle, yaw_deg=0.0) # Yaw em 0.0 (reto, sem virar)
    configurar_parametro(vehicle, 'WP_YAW_BEHAVIOR', 0)
    
    alt_atual = vehicle.location.global_relative_frame.alt
    if alt_atual < 0.3:
        print("[*] Drone no solo. Decolando para 1.5m de segurança...")
        arm_and_takeoff(vehicle, 1.50)
    
    # 1. Navega até a armadilha a 1.5m
    print("[*] Movendo até o topo da armadilha a 1.50m...")
    ir_para_posicao_precisa(vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, 1.50)
    
    # 2. Desce suavemente para a altitude final (0.35m)
    print(f"[*] Descendo para a altitude de alinhamento fino ({ALTURA_ALVO:.2f}m)...")
    ir_para_posicao_precisa(vehicle, ALVO_NORTE_NED, ALVO_LESTE_NED, ALTURA_ALVO, timeout=20.0)
    
    print(f"\n[+] Drone posicionado a {ALTURA_ALVO:.2f}m sobre a armadilha!")
    print(" -> Olhe no Gazebo 3D para verificar a posição da garra.")
    print(" -> Use W/A/S/D na janela OpenCV para fazer micro-ajustes se necessário.")
    print(" -> Pressione [ESPACO] na janela quando a garra estiver perfeitamente sobre o gancho.\n")
    
    node = CalibradorAoVivoNode(vehicle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        vehicle.close()

if __name__ == '__main__':
    main()
