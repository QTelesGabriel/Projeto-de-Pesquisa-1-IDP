# visao/rastreador_fino.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import time
import math
from ultralytics import YOLO

from controle_voo.movimento import enviar_velocidade

class RastreadorFinoYOLO(Node):
    def __init__(self, vehicle):
        super().__init__('rastreador_fino_yolo')
        self.vehicle = vehicle
        self.bridge = CvBridge()
        
        # 1. Carrega o modelo YOLO da Fase 2
        # ATENÇÃO: Atualize o nome da pasta do seu novo modelo aqui!
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', '..', 'modelos_IA', 'YOLOv11n_Pose_Armadilha', 'weights', 'best.pt')
        self.get_logger().info(f"Carregando modelo YOLO FASE 2 de: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)
        
        # 2. Configurações de Controle (Ajuste Fino)
        # Ganhos bem menores que a Fase 1 para evitar oscilações brutas de perto
        self.kp = 0.2  
        self.kd = 0.05 
        
        # Ganhos para a rotação (Yaw)
        self.kp_yaw = 0.4
        self.kd_yaw = 0.1
        
        self.erro_x_anterior = 0.0
        self.erro_y_anterior = 0.0
        self.erro_yaw_anterior = 0.0
        self.ultimo_tempo = None
        
        # 3. Geometria Física e Máquina de Estados (Passo 4 e 5)
        self.altura_gaiola = 0.36 # Altura exata extraída do STL
        self.ponto_critico = 0.50 # Altitude RELATIVA em que inicia a Descida Cega (Blind Drop)
        self.altitude_captura = 0.05 # Altitude RELATIVA que consideramos "Toque" na gaiola (Captura)
        
        self.erro_final_x = None
        self.erro_final_y = None
        self.iniciou_blind_drop = False
        
        # Filtro EKF para limpar ruídos da visão a curta distância
        try:
            from visao.filtro import FiltroAlvoEKF
            self.filtro = FiltroAlvoEKF(dt_inicial=0.1)
            self.usar_filtro = True
        except ImportError:
            self.usar_filtro = False
        
        # Parâmetros intrínsecos da câmera
        self.fx = 1124.0
        self.fy = 1124.0
        self.cx = 960.0
        self.cy = 540.0
        
        self.subscription = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.info_subscription = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        self.nome_janela = "Visao VANT - Fase 2 (Micro)"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("Nó da Fase 2 iniciado! Hovering a 2 metros e buscando ponto de captura...")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

    def image_callback(self, msg):
        tempo_atual = time.time()
        if self.ultimo_tempo is not None:
            if (tempo_atual - self.ultimo_tempo) < 0.05:
                return # Limita a ~20 FPS
            dt = tempo_atual - self.ultimo_tempo
        else:
            dt = 0.05
        self.ultimo_tempo = tempo_atual
        
        # Calcula a altitude relativa exata (distância da lente até o topo da gaiola)
        altitude_bruta = max(0.0, self.vehicle.location.global_relative_frame.alt)
        altitude_relativa = max(0.0, altitude_bruta - self.altura_gaiola)
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # --- PASSO 5: VERIFICAÇÃO DE CAPTURA ---
        if altitude_relativa <= self.altitude_captura:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0) # Desliga os motores e rotação
            self.get_logger().info(f"CAPTURA BEM SUCEDIDA! Altura de toque atingida.")
            
            # Cálcula o erro absoluto final (RMSE)
            if self.erro_final_x is not None:
                rmse = (self.erro_final_x**2 + self.erro_final_y**2)**0.5
                self.get_logger().info(f"========== RESULTADO PARA O TCC ==========")
                self.get_logger().info(f"ERRO FINAL (RMSE) NO PONTO DE CORTE: {rmse:.4f} metros")
                self.get_logger().info(f"==========================================")
                
            raise SystemExit
            
        # --- PASSO 4: BLIND DROP ATIVO? ---
        if self.iniciou_blind_drop:
            vel_z = 0.25 # Desce constante e firme
            enviar_velocidade(self.vehicle, 0.0, 0.0, vel_z, 0.0) # X, Y e Yaw zerados!
            
            texto_status = "BLIND DROP ATIVO"
            cv2.putText(cv_image, f"Alt Rel: {altitude_relativa:.2f}m | {texto_status}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow(self.nome_janela, cv_image)
            cv2.waitKey(1)
            return

        # --- PASSO 3: FUNIL VISUAL (Visão Ativa - POSE) ---
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.7)
        
        centro_x, centro_y = None, None
        alcas = []
        
        # Desenha a mira (Centro da tela)
        cv2.line(cv_image, (int(self.cx) - 20, int(self.cy)), (int(self.cx) + 20, int(self.cy)), (255, 255, 255), 2)
        cv2.line(cv_image, (int(self.cx), int(self.cy) - 20), (int(self.cx), int(self.cy) + 20), (255, 255, 255), 2)
        
        if len(resultados[0].boxes) > 0:
            # Desenha a Bounding Box inteira da armadilha
            x1, y1, x2, y2 = map(int, resultados[0].boxes[0].xyxy[0].cpu().numpy())
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
            
            # Extrai os Keypoints (Pose)
            if resultados[0].keypoints is not None:
                kpts = resultados[0].keypoints.xy[0].cpu().numpy()
                
                # Se o modelo detectou os keypoints
                if len(kpts) >= 4:
                    # Filtra apenas pontos válidos (não ocluídos)
                    pts_validos = [k for k in kpts[:4] if k[0] > 0 and k[1] > 0]
                    
                    if len(pts_validos) >= 2:
                        # Inteligência Geométrica: O Alvo (Gancho) está sempre no centro físico da armadilha.
                        # Logo, o ponto que estiver mais próximo do centroide geométrico de todos os pontos é o Alvo.
                        cx_geo = sum([k[0] for k in pts_validos]) / len(pts_validos)
                        cy_geo = sum([k[1] for k in pts_validos]) / len(pts_validos)
                        
                        idx_alvo = 0
                        menor_dist = float('inf')
                        for i, p in enumerate(pts_validos):
                            dist = (p[0] - cx_geo)**2 + (p[1] - cy_geo)**2
                            if dist < menor_dist:
                                menor_dist = dist
                                idx_alvo = i
                                
                        # Aloca o Alvo (Centro para Translação)
                        px_alvo, py_alvo = pts_validos[idx_alvo]
                        centro_x, centro_y = px_alvo, py_alvo
                        cv2.circle(cv_image, (int(px_alvo), int(py_alvo)), 8, (0, 0, 255), -1)
                        cv2.putText(cv_image, "ALVO", (int(px_alvo)+10, int(py_alvo)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                        
                        # Aloca os demais pontos como Alças (Para Rotação Yaw)
                        for i, p in enumerate(pts_validos):
                            if i != idx_alvo:
                                alcas.append((p[0], p[1]))
                                cv2.circle(cv_image, (int(p[0]), int(p[1])), 6, (255, 0, 255), -1)
                                cv2.putText(cv_image, "ALCA", (int(p[0])+10, int(p[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,255), 1)
        
        # Converte pixel para metros (medida bruta)
        x_m_bruto, y_m_bruto = None, None
        if centro_x is not None and centro_y is not None:
            x_m_bruto = (centro_x - self.cx) * altitude_relativa / self.fx
            y_m_bruto = (centro_y - self.cy) * altitude_relativa / self.fy
            
        # Filtro EKF (para limpar saltos e lidar com frames perdidos)
        if self.usar_filtro:
            erro_x, erro_y = self.filtro.atualizar(x_m_bruto, y_m_bruto, dt)
        else:
            erro_x, erro_y = x_m_bruto, y_m_bruto
            
        if erro_x is not None and erro_y is not None:
            self.erro_final_x = erro_x
            self.erro_final_y = erro_y
            
            # PD de Translação (X e Y)
            derivada_x = (erro_x - self.erro_x_anterior) / dt
            derivada_y = (erro_y - self.erro_y_anterior) / dt
            self.erro_x_anterior = erro_x
            self.erro_y_anterior = erro_y
            
            vel_x = (erro_x * self.kp + derivada_x * self.kd)
            vel_y = (erro_y * self.kp + derivada_y * self.kd)
            
            max_vel = 0.5
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # PD para Rotação (Yaw Rate)
            yaw_rate = 0.0
            menor_erro_yaw = float('inf')
            
            if len(alcas) > 0 and centro_x is not None:
                for alca in alcas:
                    dx = alca[0] - centro_x
                    dy = alca[1] - centro_y
                    angulo_alca = math.atan2(dy, dx)
                    
                    # Erro: A câmera é um sistema inverso. Se queremos que a alça suba para o topo (-PI/2),
                    # a conta correta para o Drone girar no sentido certo é (Atual - Alvo).
                    erro_angular = angulo_alca - (-math.pi / 2.0)
                    erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
                    
                    if abs(erro_angular) < abs(menor_erro_yaw):
                        menor_erro_yaw = erro_angular
                        
                if menor_erro_yaw != float('inf'):
                    derivada_yaw = (menor_erro_yaw - self.erro_yaw_anterior) / dt
                    self.erro_yaw_anterior = menor_erro_yaw
                    
                    yaw_rate = menor_erro_yaw * self.kp_yaw + derivada_yaw * self.kd_yaw
                    max_yaw = 0.5
                    yaw_rate = max(-max_yaw, min(max_yaw, yaw_rate))
            else:
                menor_erro_yaw = self.erro_yaw_anterior # Mantém a memória para o teste abaixo
            
            # --- LÓGICA DE TRANSIÇÃO PARA BLIND DROP ---
            erro_xy_atual = (erro_x**2 + erro_y**2)**0.5
            
            if altitude_relativa <= self.ponto_critico:
                # O drone só aciona a queda cega se estiver PERFEITAMENTE alinhado e TENDO VISTO o alvo neste frame
                if erro_xy_atual < 0.03 and abs(menor_erro_yaw) < 0.08 and centro_x is not None: 
                    self.iniciou_blind_drop = True
                    self.get_logger().info("ALINHAMENTO PERFEITO ATINGIDO! Iniciando Blind Drop.")
                    return # Próximo frame fará a queda cega
                else:
                    # Trava a altitude (hover) para terminar de alinhar
                    vel_z = 0.0
                    texto_status = "HOVER: ALINHAMENTO FINAL PRE-DROP"
                    cor_status = (0, 255, 255) # Amarelo de alerta
            else:
                # Acima do ponto crítico, a histerese de descida normal opera
                if erro_xy_atual < 0.12:
                    vel_z = 0.2
                    texto_status = "ALINHAMENTO 3D ATIVO + DESCENDO"
                    cor_status = (0, 255, 0)
                else:
                    vel_z = 0.0
                    texto_status = "ALINHANDO XY e YAW..."
                    cor_status = (0, 165, 255)
            
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z, yaw_rate)
        else:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
            texto_status = "ALVO PERDIDO (Filtro Aguardando)"
            cor_status = (0, 0, 255)
            
        cv2.putText(cv_image, f"Alt Rel: {altitude_relativa:.2f}m | {texto_status}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor_status, 2)
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)
