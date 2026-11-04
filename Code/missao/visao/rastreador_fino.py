# visao/rastreador_fino.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import time
import math
import numpy as np
from ultralytics import YOLO

from controle_voo.movimento import enviar_velocidade
from controle_voo.gimbal import apontar_gimbal_nadir

class RastreadorFinoYOLO(Node):
    def __init__(self, vehicle):
        super().__init__('rastreador_fino_yolo')
        self.vehicle = vehicle
        self.bridge = CvBridge()
        
        # 1. Carrega o modelo YOLO da Fase 2 (Melhor treinado: YOLOv10n - mAP50-95 de 89.08% - 2 Classes: 0: alca, 1: armadilha)
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', '..', 'modelos_IA', 'YOLOv10n_Fase2_Armadilha_Alcas', 'weights', 'best.pt')
        self.get_logger().info(f"Carregando modelo YOLO FASE 2 de: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)
        
        # 2. Configurações de Controle (Ajuste Fino)
        # Ganhos bem menores que a Fase 1 para evitar oscilações brutas de perto
        self.kp = 0.15  
        self.kd = 0.05
        
        # Ganhos para a rotação (Yaw)
        self.kp_yaw = 0.4
        self.kd_yaw = 0.1
        
        self.erro_x_anterior = 0.0
        self.erro_y_anterior = 0.0
        self.erro_yaw_anterior = 0.0
        self.ultimo_tempo = None
        
        # 3. Geometria Física e Máquina de Estados (4 Etapas)
        self.altura_gaiola = 0.317         # Altura exata da tampa da gaiola (sem alças, extraída do STL)
        self.altura_camera_offset = 0.320  # Distância dos pés de pouso do drone até a lente da câmera (m)
        
        # Proporção do Offset do Alvo (Em relação ao tamanho da Bounding Box)
        # Valores calculados analiticamente a partir da geometria CAD/SDF do chassi e do gimbal:
        # X: Câmera perfeitamente centrada no eixo lateral (desvio < 0.5 mm) -> offset_pct_x = 0.0000
        # Y: Mount do Gimbal a +0.04216m do centro do drone / Bounding Box de 0.2550m -> offset_pct_y = 0.1653
        self.offset_pct_x = 0.0000
        self.offset_pct_y = 0.1653
        
        # Parâmetros das 4 Etapas da Fase 2
        self.alt_yaw = 1.50                # Etapa 1 -> Etapa 2: Hover a 1.50m para alinhamento de Yaw (subido 10 cm)
        # Ângulo alvo compensado para a assimetria física do CAD (+93.37° em vez de 90.00°)
        # Isso garante que a carcaça e a linha da base fiquem perfeitamente ortogonais ao drone
        self.angulo_alvo = math.radians(93.37)
        self.margem_erro_yaw_deg = 1.0     # Margem estrita de erro angular no Yaw (máximo 1.0°)
        self.tempo_yaw_ok_necessario = 0.3  # 0.30s contínuos sustentados <= 1.0°
        self.ultimo_erro_yaw_deg = None    # Memoriza o último erro de Yaw para exibição contínua
        self.tempo_sem_3_alcas = None      # Temporizador para detectar alça oculta por sombra do drone
        self.timeout_sombra_alca = 3.0     # 3.0s sem as 3 alças para forçar rotação
        self.vel_yaw_sombra = 0.25         # Velocidade angular forçada (rad/s) para fugir da sombra
        self.ultimo_log_sombra = 0.0       # Controle de frequência de logs de alerta
        self.alt_final = 0.35              # Etapa 3A -> Etapa 3B: Hover a 0.35m para ajuste fino no offset
        self.margem_erro_cravado = 0.005   # Margem reduzida para 5 mm (0.005m) de erro no offset (ajuste fino rigoroso)
        self.tempo_cravado_necessario = 1.5 # 1.5 segundos sustentados continuamente <= 5 mm (30 frames contínuos)
        self.vel_descida_cega = 0.28       # Velocidade de descida rápida/firme (m/s) para anular deriva lateral de efeito solo
        self.altitude_captura = 0.05       # Altitude bruta (m) dos pés do drone para desarme/toque no solo
        
        self.erro_final_x = None
        self.erro_final_y = None
        self.iniciou_blind_drop = False
        
        # Variáveis da Máquina de Estados de Descida
        self.rotacao_concluida = False
        self.tempo_yaw_ok = None
        self.tempo_cravado = None
        self.inicio_hover_etapa3b = None   # Marca início do hover a 0.35m para timeout de segurança
        self.timeout_etapa3b = 8.0         # Após 8.0s em 3B, aceita tolerância de segurança (15 mm) para nunca travar
        self.etapa_atual_nome = "ETAPA 1/4: APROXIMACAO INICIAL"
        
        # Filtro EMA para Bounding Box
        self.ema_x1 = None
        self.ema_y1 = None
        self.ema_x2 = None
        self.ema_y2 = None
        self.ema_alpha = 0.5 # 0.5 suaviza bem sem adicionar muito atraso
        
        # Filtro EKF para limpar ruídos da visão a curta distância
        try:
            from visao.filtro import FiltroAlvoEKF, FiltroAnguloEKF
            self.filtro = FiltroAlvoEKF(dt_inicial=0.1)
            self.filtro_p0 = FiltroAlvoEKF(dt_inicial=0.1)
            self.filtro_p1 = FiltroAlvoEKF(dt_inicial=0.1)
            self.filtro_p2 = FiltroAlvoEKF(dt_inicial=0.1)
            self.filtro_angulo = FiltroAnguloEKF(dt_inicial=0.1)
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

    def alcas_sobrepostas(self, box_a, box_b, altitude_relativa):
        """
        Verifica se duas caixas de alça correspondem à mesma alça física
        (por sobreposição de área IoU/IoA ou por proximidade física excessiva).
        """
        # 1. Checagem de sobreposição geométrica de caixas (IoU / IoA)
        ix1 = max(box_a[0], box_b[0])
        iy1 = max(box_a[1], box_b[1])
        ix2 = min(box_a[2], box_b[2])
        iy2 = min(box_a[3], box_b[3])
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter_area = iw * ih
        
        if inter_area > 0:
            area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
            area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
            union_area = area_a + area_b - inter_area
            iou = inter_area / union_area if union_area > 0 else 0.0
            ioa = inter_area / min(area_a, area_b) if min(area_a, area_b) > 0 else 0.0
            # Alças distintas no CAD nunca se tocam (IoU=0). Qualquer overlap indica duplicação.
            if iou > 0.05 or ioa > 0.10:
                return True

        # 2. Checagem de distância física mínima entre centros no plano da armadilha
        ca_x = (box_a[0] + box_a[2]) / 2.0
        ca_y = (box_a[1] + box_a[3]) / 2.0
        cb_x = (box_b[0] + box_b[2]) / 2.0
        cb_y = (box_b[1] + box_b[3]) / 2.0
        dist_px = ((ca_x - cb_x)**2 + (ca_y - cb_y)**2)**0.5
        dist_m = dist_px * max(0.01, altitude_relativa) / self.fx
        
        # As alças físicas da gaiola têm separação mínima de 10 a 18 cm entre si.
        # Se os centros estiverem a menos de 6 cm (0.06m), é a mesma alça duplicada.
        if dist_m < 0.06:
            return True
            
        return False

    def image_callback(self, msg):
        tempo_atual = time.time()
        if self.ultimo_tempo is not None:
            if (tempo_atual - self.ultimo_tempo) < 0.05:
                return # Limita a ~20 FPS (Exigência física/visual do sistema)
            dt = tempo_atual - self.ultimo_tempo
        else:
            dt = 0.05
        self.ultimo_tempo = tempo_atual
        
        # Calcula a distância óptica exata da lente da câmera até o plano superior da gaiola
        altitude_bruta = max(0.0, self.vehicle.location.global_relative_frame.alt)
        # Distância óptica real (Pinhole Z-Depth):
        # z_camera = altitude_bruta + 0.320m (offset dos pés até a lente); z_alvo = 0.317m
        altitude_relativa = max(0.005, (altitude_bruta + self.altura_camera_offset) - self.altura_gaiola) # Proteção contra divisão por zero
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # --- ETAPA 4: DESCIDA FINAL (BLIND DROP) E VERIFICAÇÃO DE CAPTURA ---
        if self.iniciou_blind_drop:
            # A verificação de toque e corte só atua DEPOIS que o Blind Drop foi formalmente disparado na Etapa 3B
            if altitude_bruta <= self.altitude_captura or altitude_relativa <= self.altitude_captura:
                enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0) # Desliga os motores e rotação
                self.get_logger().info(f"CAPTURA BEM SUCEDIDA! Altura de toque atingida.")
                
                # Calcula o erro absoluto final (RMSE)
                if self.erro_final_x is not None:
                    rmse = (self.erro_final_x**2 + self.erro_final_y**2)**0.5
                    self.get_logger().info(f"========== RESULTADO PARA O TCC ==========")
                    self.get_logger().info(f"ERRO FINAL (RMSE) NO PONTO DE CORTE: {rmse:.4f} metros ({rmse*1000.0:.1f} mm)")
                    self.get_logger().info(f"==========================================")
                    
                raise SystemExit

            vel_z = self.vel_descida_cega # Desce suave e constante (0.12 m/s)
            enviar_velocidade(self.vehicle, 0.0, 0.0, vel_z, 0.0) # X, Y e Yaw zerados!
            
            # Desliga a câmera (fecha a janela) simulando perda de feed de vídeo
            try:
                cv2.destroyWindow(self.nome_janela)
            except:
                pass
                
            return

        # --- PERCEPÇÃO VISUAL (YOLO 2 Classes) ---
        # ISOLAMENTO DE IMAGEM: O YOLO recebe um clone intocado (cópia limpa) da câmera.
        # Nenhuma alteração gráfica do OpenCV contamina a entrada do modelo.
        img_yolo = cv_image.copy()
        resultados = self.yolo.predict(img_yolo, verbose=False, conf=0.5)
        
        centro_x, centro_y = None, None
        alcas_pts = []
        erro_angular_deg = None
        yaw_rate = 0.0
        
        # 1. Desenha a mira central (Eixo óptico da Câmera / Centro do Drone)
        cv2.line(cv_image, (int(self.cx) - 25, int(self.cy)), (int(self.cx) + 25, int(self.cy)), (255, 255, 255), 2)
        cv2.line(cv_image, (int(self.cx), int(self.cy) - 25), (int(self.cx), int(self.cy) + 25), (255, 255, 255), 2)
        cv2.circle(cv_image, (int(self.cx), int(self.cy)), 4, (255, 255, 255), -1)
        cv2.putText(cv_image, "MIRA (Centro Camera)", (int(self.cx) + 12, int(self.cy) - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        
        boxes = resultados[0].boxes
        if len(boxes) > 0:
            # Separa detecções por classe (0: alca, 1: armadilha)
            boxes_armadilha = [b for b in boxes if int(b.cls[0].item()) == 1]
            boxes_alca = [b for b in boxes if int(b.cls[0].item()) == 0]
            
            # 2. Analisa a Armadilha (Gaiola inteira) e calcula o Alvo Virtual com Offset
            if len(boxes_armadilha) > 0:
                best_armadilha = max(boxes_armadilha, key=lambda b: float(b.conf[0].item()))
                x1_raw, y1_raw, x2_raw, y2_raw = map(float, best_armadilha.xyxy[0].cpu().numpy())
                
                # Filtro EMA para suavizar a bounding box da armadilha
                if self.ema_x1 is None:
                    self.ema_x1, self.ema_y1, self.ema_x2, self.ema_y2 = x1_raw, y1_raw, x2_raw, y2_raw
                else:
                    self.ema_x1 = (self.ema_alpha * x1_raw) + ((1 - self.ema_alpha) * self.ema_x1)
                    self.ema_y1 = (self.ema_alpha * y1_raw) + ((1 - self.ema_alpha) * self.ema_y1)
                    self.ema_x2 = (self.ema_alpha * x2_raw) + ((1 - self.ema_alpha) * self.ema_x2)
                    self.ema_y2 = (self.ema_alpha * y2_raw) + ((1 - self.ema_alpha) * self.ema_y2)
                    
                x1, y1, x2, y2 = int(self.ema_x1), int(self.ema_y1), int(self.ema_x2), int(self.ema_y2)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
                
                box_w = x2 - x1
                box_h = y2 - y1
                box_cx = x1 + box_w / 2.0
                box_cy = y1 + box_h / 2.0
                cv2.circle(cv_image, (int(box_cx), int(box_cy)), 5, (255, 165, 0), -1)
                
                # Alvo Virtual com Offset Proporcional calibrado
                alvo_x = box_cx + (box_w * self.offset_pct_x)
                alvo_y = box_cy - (box_h * self.offset_pct_y)
                
                # O alinhamento no Alvo Virtual com Offset atua após a trava de Yaw (Etapa 3)
                if self.rotacao_concluida:
                    centro_x, centro_y = alvo_x, alvo_y
                    cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 7, (0, 0, 255), -1)
                    cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 13, (0, 0, 255), 2)
                    cv2.putText(cv_image, "ALVO (OFFSET)", (int(alvo_x) + 16, int(alvo_y)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    # Durante Etapas 1 e 2: mira no centro geométrico da armadilha
                    centro_x, centro_y = box_cx, box_cy
                    cv2.putText(cv_image, "CENTRO GAIOLA", (int(box_cx) + 10, int(box_cy)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 165, 0), 1)
                    # Desenha prévia discreta do offset
                    cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 5, (128, 128, 128), -1)
                            
            # 3. Analisa as Alças e Elimina Detecções Duplicadas / Sobrepostas
            sorted_alcas = sorted(boxes_alca, key=lambda b: float(b.conf[0].item()), reverse=True)
            alcas_distintas = []
            
            for b in sorted_alcas:
                b_coords = list(map(int, b.xyxy[0].cpu().numpy()))
                # Verifica se sobrepõe com alguma alça já aceita
                sobreposicao = False
                for _, u_coords in alcas_distintas:
                    if self.alcas_sobrepostas(b_coords, u_coords, altitude_relativa):
                        sobreposicao = True
                        break
                
                if not sobreposicao:
                    alcas_distintas.append((b, b_coords))
                else:
                    # Alça duplicada detectada sobre a mesma estrutura física
                    if not self.rotacao_concluida:
                        cv2.rectangle(cv_image, (b_coords[0], b_coords[1]), (b_coords[2], b_coords[3]), (0, 0, 255), 1)
                        cv2.putText(cv_image, "DUPLICADA", (b_coords[0], b_coords[1] - 4), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
            
            # Seleciona apenas as 3 alças estritamente distintas de maior confiança
            for b, coords in alcas_distintas[:3]:
                pcx = (coords[0] + coords[2]) / 2.0
                pcy = (coords[1] + coords[3]) / 2.0
                alcas_pts.append((pcx, pcy))
                # Só desenha as marcações das alças enquanto o Yaw NÃO estiver travado
                if not self.rotacao_concluida:
                    cv2.rectangle(cv_image, (coords[0], coords[1]), (coords[2], coords[3]), (0, 220, 255), 1)
                    cv2.circle(cv_image, (int(pcx), int(pcy)), 4, (0, 220, 255), -1)
                
            agora = time.time()
            
            # Aplicação do Filtro de Kalman nas 3 Alças e no Ângulo
            if len(alcas_pts) == 3:
                self.tempo_sem_3_alcas = None # Todas as 3 alças encontradas com sucesso!
                pts = [np.array(p, dtype=float) for p in alcas_pts]
                d01 = float(np.linalg.norm(pts[0] - pts[1]))
                d12 = float(np.linalg.norm(pts[1] - pts[2]))
                d20 = float(np.linalg.norm(pts[2] - pts[0]))
                
                # Diferença de comprimento dos lados adjacentes para cada candidato a vértice
                diff0 = abs(d01 - d20)
                diff1 = abs(d01 - d12)
                diff2 = abs(d12 - d20)
                
                min_diff = min(diff0, diff1, diff2)
                if min_diff == diff0:
                    apex_raw, cand_b1, cand_b2 = pts[0], pts[1], pts[2]
                elif min_diff == diff1:
                    apex_raw, cand_b1, cand_b2 = pts[1], pts[0], pts[2]
                else:
                    apex_raw, cand_b1, cand_b2 = pts[2], pts[0], pts[1]
                    
                # Separa consistentemente b1 (esquerda) e b2 (direita) pelo produto vetorial com o eixo
                mid_raw = (cand_b1 + cand_b2) / 2.0
                u_raw = apex_raw - mid_raw
                v_cand = cand_b1 - mid_raw
                cross = u_raw[0] * v_cand[1] - u_raw[1] * v_cand[0]
                if cross > 0:
                    b1_raw, b2_raw = cand_b1, cand_b2
                else:
                    b1_raw, b2_raw = cand_b2, cand_b1
                    
                # Aplica o Filtro de Kalman (FiltroAlvoEKF) individualmente nos 3 pontos das alças
                if self.usar_filtro:
                    x_m_apex = (apex_raw[0] - self.cx) * altitude_relativa / self.fx
                    y_m_apex = (apex_raw[1] - self.cy) * altitude_relativa / self.fy
                    x_m_b1 = (b1_raw[0] - self.cx) * altitude_relativa / self.fx
                    y_m_b1 = (b1_raw[1] - self.cy) * altitude_relativa / self.fy
                    x_m_b2 = (b2_raw[0] - self.cx) * altitude_relativa / self.fx
                    y_m_b2 = (b2_raw[1] - self.cy) * altitude_relativa / self.fy
                    
                    f_x_apex, f_y_apex = self.filtro_p0.atualizar(x_m_apex, y_m_apex, dt)
                    f_x_b1, f_y_b1 = self.filtro_p1.atualizar(x_m_b1, y_m_b1, dt)
                    f_x_b2, f_y_b2 = self.filtro_p2.atualizar(x_m_b2, y_m_b2, dt)
                    
                    if f_x_apex is not None and f_x_b1 is not None and f_x_b2 is not None:
                        px_apex = (f_x_apex * self.fx / altitude_relativa) + self.cx
                        py_apex = (f_y_apex * self.fy / altitude_relativa) + self.cy
                        px_b1 = (f_x_b1 * self.fx / altitude_relativa) + self.cx
                        py_b1 = (f_y_b1 * self.fy / altitude_relativa) + self.cy
                        px_b2 = (f_x_b2 * self.fx / altitude_relativa) + self.cx
                        py_b2 = (f_y_b2 * self.fy / altitude_relativa) + self.cy
                        apex_pt = np.array([px_apex, py_apex])
                        b1_pt = np.array([px_b1, py_b1])
                        b2_pt = np.array([px_b2, py_b2])
                    else:
                        apex_pt, b1_pt, b2_pt = apex_raw, b1_raw, b2_raw
                else:
                    apex_pt, b1_pt, b2_pt = apex_raw, b1_raw, b2_raw
                    
                mid_base = (b1_pt + b2_pt) / 2.0
                vec = apex_pt - mid_base
                dx, dy = float(vec[0]), float(vec[1])
                angulo_atual_bruto = math.atan2(dy, dx)
                
                # Aplica o Filtro de Kalman no Ângulo
                if self.usar_filtro:
                    angulo_atual = self.filtro_angulo.atualizar(angulo_atual_bruto, dt)
                    if angulo_atual is None:
                        angulo_atual = angulo_atual_bruto
                else:
                    angulo_atual = angulo_atual_bruto
                    
                # Requisito: Base para cima e vértice para baixo (+93.37° compensando assimetria do CAD)
                angulo_alvo = self.angulo_alvo
                erro_angular = (angulo_atual - angulo_alvo + math.pi) % (2.0 * math.pi) - math.pi
                erro_angular_deg = math.degrees(erro_angular)
                
                # Desenho do triângulo e dos 3 pontos das alças FILTRADOS pelo Kalman (EKF)
                if not self.rotacao_concluida:
                    # Alças filtradas com círculos e anéis EKF
                    # Vértice em magenta (Ponto Apex filtrado)
                    cv2.circle(cv_image, (int(apex_pt[0]), int(apex_pt[1])), 8, (255, 0, 255), -1)
                    cv2.circle(cv_image, (int(apex_pt[0]), int(apex_pt[1])), 13, (255, 0, 255), 2)
                    cv2.putText(cv_image, "EKF P0", (int(apex_pt[0]) + 15, int(apex_pt[1]) + 4), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 0, 255), 1)
                    
                    # Pontos da Base em ciano (Pontos Base 1 e 2 filtrados)
                    cv2.circle(cv_image, (int(b1_pt[0]), int(b1_pt[1])), 6, (255, 255, 0), -1)
                    cv2.circle(cv_image, (int(b1_pt[0]), int(b1_pt[1])), 11, (255, 255, 0), 2)
                    cv2.putText(cv_image, "EKF P1", (int(b1_pt[0]) - 55, int(b1_pt[1]) + 4), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 0), 1)
                                
                    cv2.circle(cv_image, (int(b2_pt[0]), int(b2_pt[1])), 6, (255, 255, 0), -1)
                    cv2.circle(cv_image, (int(b2_pt[0]), int(b2_pt[1])), 11, (255, 255, 0), 2)
                    cv2.putText(cv_image, "EKF P2", (int(b2_pt[0]) + 15, int(b2_pt[1]) + 4), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 0), 1)
                    
                    # Linhas do triângulo filtrado
                    cv2.line(cv_image, (int(b1_pt[0]), int(b1_pt[1])), (int(b2_pt[0]), int(b2_pt[1])), (255, 255, 0), 2)
                    cv2.line(cv_image, (int(apex_pt[0]), int(apex_pt[1])), (int(b1_pt[0]), int(b1_pt[1])), (0, 255, 0), 2)
                    cv2.line(cv_image, (int(apex_pt[0]), int(apex_pt[1])), (int(b2_pt[0]), int(b2_pt[1])), (0, 255, 0), 2)
                    
                    # Seta de orientação calculada pelo ângulo filtrado
                    cv2.arrowedLine(cv_image, (int(mid_base[0]), int(mid_base[1])), 
                                    (int(apex_pt[0]), int(apex_pt[1])), (0, 255, 255), 2, tipLength=0.25)
                    cv2.putText(cv_image, "ALCAS FILTRADAS (EKF)", (int(mid_base[0]) - 85, int(mid_base[1]) - 14), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)
                                
                # Controle de Yaw (Ativo estritamente na Etapa 2: altitude_bruta <= 1.40m)
                if not self.rotacao_concluida:
                    if altitude_bruta <= self.alt_yaw:
                        # ETAPA 2: Ajuste e trava de Yaw a 1.40m
                        if abs(erro_angular_deg) <= self.margem_erro_yaw_deg:
                            if self.tempo_yaw_ok is None:
                                self.tempo_yaw_ok = agora
                            elif (agora - self.tempo_yaw_ok) >= self.tempo_yaw_ok_necessario:
                                self.rotacao_concluida = True
                                yaw_rate = 0.0
                                if self.usar_filtro:
                                    self.filtro.iniciado = False  # Reinicia o EKF para ancorar suave no Offset
                                self.get_logger().info(f"[+] ETAPA 2 CONCLUIDA: Yaw travado com erro {erro_angular_deg:+.2f}° (Meta <={self.margem_erro_yaw_deg:.1f}°)! Avancando para Etapa 3 (Descida no Offset).")
                        else:
                            self.tempo_yaw_ok = None
                            
                        derivada_yaw = (erro_angular - self.erro_yaw_anterior) / dt
                        self.erro_yaw_anterior = erro_angular
                        yaw_rate = erro_angular * self.kp_yaw + derivada_yaw * self.kd_yaw
                        max_yaw = 0.4
                        yaw_rate = max(-max_yaw, min(max_yaw, yaw_rate))
                    else:
                        # ETAPA 1: Acima de 1.40m o Yaw não gira
                        yaw_rate = 0.0
                        self.tempo_yaw_ok = None
                else:
                    # Yaw travado permanentemente após alinhamento
                    yaw_rate = 0.0
            else:
                # Menos de 3 alças encontradas
                if not self.rotacao_concluida:
                    self.tempo_yaw_ok = None
                    if altitude_bruta <= self.alt_yaw:
                        # ETAPA 2: Monitora se alças estão cobertas pela sombra do drone
                        if self.tempo_sem_3_alcas is None:
                            self.tempo_sem_3_alcas = agora
                        tempo_sem_alcas = agora - self.tempo_sem_3_alcas
                        
                        if tempo_sem_alcas >= self.timeout_sombra_alca:
                            # Passou de 3 segundos sem encontrar as 3 alças: FORÇA ROTAÇÃO ANTI-SOMBRA
                            yaw_rate = self.vel_yaw_sombra
                            if (agora - self.ultimo_log_sombra) > 2.0:
                                self.ultimo_log_sombra = agora
                                self.get_logger().warn(f"[!] DESVIO DE SOMBRA: Apenas {len(alcas_pts)}/3 alcas visiveis ha {tempo_sem_alcas:.1f}s. Forcando rotacao do drone para desviar da sombra!")
                        else:
                            yaw_rate = 0.0
                    else:
                        yaw_rate = 0.0
                        self.tempo_sem_3_alcas = None
                else:
                    yaw_rate = 0.0
                    self.tempo_sem_3_alcas = None
                    
                if self.usar_filtro:
                    self.filtro_p0.atualizar(None, None, dt)
                    self.filtro_p1.atualizar(None, None, dt)
                    self.filtro_p2.atualizar(None, None, dt)
                    self.filtro_angulo.atualizar(None, dt)
                    
        # Converte o Alvo de pixel para metros físicos (PBVS)
        x_m_bruto, y_m_bruto = None, None
        if centro_x is not None and centro_y is not None:
            x_m_bruto = (centro_x - self.cx) * altitude_relativa / self.fx
            y_m_bruto = (centro_y - self.cy) * altitude_relativa / self.fy
            
        # Filtro EKF para o Alvo (Lidando com Jitter)
        if self.usar_filtro:
            erro_x, erro_y = self.filtro.atualizar(x_m_bruto, y_m_bruto, dt)
            if erro_x is not None and erro_y is not None:
                # Projeta de volta para visualização na imagem
                pixel_x_filtrado = int((erro_x * self.fx / altitude_relativa) + self.cx)
                pixel_y_filtrado = int((erro_y * self.fy / altitude_relativa) + self.cy)
                cv2.circle(cv_image, (pixel_x_filtrado, pixel_y_filtrado), 6, (255, 255, 0), -1)
        else:
            erro_x, erro_y = x_m_bruto, y_m_bruto
            
        agora = time.time()
        
        if erro_x is not None and erro_y is not None:
            self.erro_final_x = erro_x
            self.erro_final_y = erro_y
            
            # Derivadas de velocidade translacional
            derivada_x = (erro_x - self.erro_x_anterior) / dt
            derivada_y = (erro_y - self.erro_y_anterior) / dt
            self.erro_x_anterior = erro_x
            self.erro_y_anterior = erro_y
            
            erro_xy_atual = (erro_x**2 + erro_y**2)**0.5
            erro_mm = erro_xy_atual * 1000.0
            
            # --- MÁQUINA DE ESTADOS DA FASE 2 (4 ETAPAS) ---
            if not self.rotacao_concluida:
                # ETAPAS ANTERIORES À TRAVA DE YAW (Centralizado na Armadilha)
                if altitude_bruta > self.alt_yaw:
                    # ETAPA 1: Aproximação inicial até 1.40m
                    vel_z = 0.20
                    yaw_rate = 0.0 # Yaw travado, não gira ainda
                    self.tempo_yaw_ok = None
                    self.tempo_cravado = None
                    max_vel = 0.4
                    kp_atual = self.kp
                    self.etapa_atual_nome = f"ETAPA 1/4: APROXIMACAO INICIAL (Descendo para {self.alt_yaw:.2f}m)"
                    cor_status = (0, 165, 255) # Laranja
                else:
                    # ETAPA 2: Hover a 1.40m para alinhamento e trava de Yaw com Kalman nas alças
                    vel_z = 0.0 # Hover estável a 1.40m
                    self.tempo_cravado = None
                    max_vel = 0.25
                    kp_atual = self.kp
                    
                    if erro_angular_deg is not None:
                        if self.tempo_yaw_ok is not None:
                            tempo_yaw_acumulado = agora - self.tempo_yaw_ok
                            self.etapa_atual_nome = f"ETAPA 2/4: AJUSTE YAW @ {self.alt_yaw:.2f}m | Erro: {erro_angular_deg:+.1f}° ({tempo_yaw_acumulado:.2f}s/{self.tempo_yaw_ok_necessario:.2f}s)"
                            cor_status = (0, 255, 0) # Verde se estiver sustentando dentro da meta
                        else:
                            self.etapa_atual_nome = f"ETAPA 2/4: AJUSTE YAW @ {self.alt_yaw:.2f}m | Erro: {erro_angular_deg:+.1f}° (Meta <={self.margem_erro_yaw_deg:.1f}°)"
                            cor_status = (0, 255, 255) # Amarelo
                    else:
                        self.tempo_yaw_ok = None
                        tempo_sem = (agora - self.tempo_sem_3_alcas) if self.tempo_sem_3_alcas is not None else 0.0
                        if tempo_sem >= self.timeout_sombra_alca:
                            self.etapa_atual_nome = f"ETAPA 2/4: DESVIO DE SOMBRA! Forcando Rotacao ({len(alcas_pts)}/3 alcas)"
                            cor_status = (255, 0, 255) # Magenta para indicar desvio de sombra
                        else:
                            self.etapa_atual_nome = f"ETAPA 2/4: BUSCANDO 3 ALCAS ({len(alcas_pts)}/3) - {tempo_sem:.1f}s/{self.timeout_sombra_alca:.1f}s"
                            cor_status = (0, 165, 255) # Laranja
            else:
                # ETAPAS APÓS TRAVA DE YAW (Centralizado no Alvo Virtual com Offset)
                yaw_rate = 0.0 # Yaw permanentemente travado
                
                if altitude_bruta > self.alt_final:
                    # ETAPA 3A: Descida centrada no Offset até alt_final (0.35m)
                    dist_descida = altitude_bruta - self.alt_final
                    # Desacelera suavemente nos últimos 25cm para amortecer a inércia e não quicar/ultrapassar
                    fator_frenagem = min(1.0, max(0.25, dist_descida / 0.25))
                    vel_z = 0.18 * fator_frenagem
                    self.tempo_cravado = None
                    self.inicio_hover_etapa3b = None
                    max_vel = 0.35
                    kp_atual = self.kp
                    self.etapa_atual_nome = f"ETAPA 3/4: DESCENDO NO OFFSET (Ate {self.alt_final:.2f}m)"
                    cor_status = (0, 220, 0) # Verde
                else:
                    # ETAPA 3B: Ajuste Fino a 0.35m com margem estrita <= 8 mm por 1.5s
                    vel_z = 0.0 # Hover a 0.35m
                    max_vel = 0.06 # Permite pequenas correções milimétricas sem ficar preso na deadband
                    kp_atual = 0.25 # Ganho proporcional responsivo para distâncias milimétricas
                    
                    if self.inicio_hover_etapa3b is None:
                        self.inicio_hover_etapa3b = agora
                    tempo_em_hover = agora - self.inicio_hover_etapa3b

                    # Critério 1: Alinhamento cravado estrito (<= 8 mm por 1.5s)
                    # Critério 2: Timeout de segurança (após 8s em hover, aceita <= 15 mm para nunca travar)
                    atingiu_cravado = (erro_xy_atual <= self.margem_erro_cravado)
                    atingiu_fallback = (tempo_em_hover >= self.timeout_etapa3b and erro_xy_atual <= 0.015)
                    
                    if atingiu_cravado or atingiu_fallback:
                        if self.tempo_cravado is None:
                            self.tempo_cravado = agora
                        tempo_acumulado = agora - self.tempo_cravado
                        meta_label = f"<={self.margem_erro_cravado*1000:.0f}mm" if atingiu_cravado else "<=15mm (Seguranca)"
                        self.etapa_atual_nome = f"ETAPA 3/4: AJUSTE FINO {meta_label} ({erro_mm:.1f}mm) - {tempo_acumulado:.1f}s/{self.tempo_cravado_necessario:.1f}s"
                        cor_status = (0, 255, 0) # Verde brilhante
                        
                        tempo_req = 0.5 if atingiu_fallback else self.tempo_cravado_necessario
                        if tempo_acumulado >= tempo_req:
                            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
                            self.iniciou_blind_drop = True
                            modo_str = "FALLBACK DE SEGURANCA" if atingiu_fallback else "PRECISAO CRAVADA"
                            self.get_logger().info(f"[+] ETAPA 3 CONCLUIDA [{modo_str}]: Alvo a {erro_mm:.2f} mm! Disparando Etapa 4 (Blind Drop a {self.vel_descida_cega} m/s).")
                            return
                    else:
                        self.tempo_cravado = None
                        self.etapa_atual_nome = f"ETAPA 3/4: AJUSTE FINO @ {self.alt_final:.2f}m | Erro: {erro_mm:.1f}mm (Meta <={self.margem_erro_cravado*1000:.0f}mm)"
                        cor_status = (0, 255, 255) # Amarelo alerta
                        
            vel_x = (erro_x * kp_atual + derivada_x * self.kd)
            vel_y = (erro_y * kp_atual + derivada_y * self.kd)
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # Envia velocidade translacional e taxa de Yaw para o drone
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z, yaw_rate)
        else:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
            self.etapa_atual_nome = "ALVO PERDIDO (Aguardando)"
            cor_status = (0, 0, 255)
            self.tempo_yaw_ok = None
            self.tempo_cravado = None
            
        # --- PAINEL DE STATUS SUPERIOR (HUD) ---
        overlay = cv_image.copy()
        cv2.rectangle(overlay, (15, 12), (920, 115), (20, 20, 20), -1)
        cv2.rectangle(overlay, (15, 12), (920, 115), (80, 80, 80), 1)
        
        # --- CAIXA DE LEGENDA VISUAL E ORIENTAÇÃO (CANTO SUPERIOR DIREITO) ---
        img_w = cv_image.shape[1]
        leg_w = 440
        leg_h = 103
        leg_x1 = max(950, img_w - leg_w - 20)
        leg_y1 = 12
        leg_x2 = leg_x1 + leg_w
        leg_y2 = leg_y1 + leg_h
        cv2.rectangle(overlay, (leg_x1, leg_y1), (leg_x2, leg_y2), (20, 20, 20), -1)
        cv2.rectangle(overlay, (leg_x1, leg_y1), (leg_x2, leg_y2), (80, 80, 80), 1)
        cv2.addWeighted(overlay, 0.75, cv_image, 0.25, 0, cv_image)
        
        # Formatação do Status e Erro de Yaw
        status_yaw_str = "TRAVADO" if self.rotacao_concluida else "ALINHANDO"
        if erro_angular_deg is not None:
            self.ultimo_erro_yaw_deg = erro_angular_deg
            erro_yaw_str = f"{erro_angular_deg:+.1f}°"
        elif self.rotacao_concluida and self.ultimo_erro_yaw_deg is not None:
            erro_yaw_str = f"{self.ultimo_erro_yaw_deg:+.1f}° (Travado)"
        else:
            erro_yaw_str = "N/A"
        
        # Textos do Painel de Status
        cv2.putText(cv_image, f"Alt Bruta: {altitude_bruta:.2f}m | Alt Rel: {altitude_relativa:.2f}m | Yaw: {status_yaw_str} (Erro: {erro_yaw_str})", 
                    (28, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(cv_image, self.etapa_atual_nome, (28, 72), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.68, cor_status, 2)
                    
        if 'erro_xy_atual' in locals():
            label_alvo = "Erro Offset" if self.rotacao_concluida else "Erro Centro Gaiola"
            cor_erro = (0, 255, 0) if (self.rotacao_concluida and erro_xy_atual <= self.margem_erro_cravado) else (0, 255, 255)
            cv2.putText(cv_image, f"{label_alvo}: {erro_mm:.1f} mm  |  Meta Fina: <= {self.margem_erro_cravado*1000:.1f} mm ({self.tempo_cravado_necessario:.1f}s)", 
                        (28, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.58, cor_erro, 2)
                        
        # Textos da Caixa de Legendas (Enxuta e com indicação clara do erro em graus)
        cv2.putText(cv_image, "=== LEGENDA & ORIENTACAO ===", (leg_x1 + 80, leg_y1 + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        cv2.putText(cv_image, "[+] Mira Drone   (*) Alvo Offset   [^] Alcas EKF", (leg_x1 + 15, leg_y1 + 53), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        
        # Linha destacada mostrando quantos graus está errado
        if erro_angular_deg is not None:
            cor_yaw_leg = (0, 255, 0) if abs(erro_angular_deg) <= self.margem_erro_yaw_deg else (0, 255, 255)
            tag_yaw = "TRAVADO" if self.rotacao_concluida else f"Meta <= {self.margem_erro_yaw_deg:.1f}°"
            texto_yaw_leg = f"Erro de Yaw: {erro_angular_deg:+.1f}° ({tag_yaw})"
        elif self.rotacao_concluida and self.ultimo_erro_yaw_deg is not None:
            cor_yaw_leg = (0, 255, 0)
            texto_yaw_leg = f"Erro de Yaw: {self.ultimo_erro_yaw_deg:+.1f}° (TRAVADO)"
        elif not self.rotacao_concluida and self.tempo_sem_3_alcas is not None and (agora - self.tempo_sem_3_alcas) >= self.timeout_sombra_alca:
            cor_yaw_leg = (255, 0, 255)
            texto_yaw_leg = f"Yaw: DESVIANDO SOMBRA ({len(alcas_pts)}/3)"
        else:
            cor_yaw_leg = (0, 165, 255)
            texto_yaw_leg = f"Erro de Yaw: Buscando alcas ({len(alcas_pts)}/3)..."
            
        cv2.putText(cv_image, texto_yaw_leg, (leg_x1 + 20, leg_y1 + 85), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, cor_yaw_leg, 2)
                    
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)

