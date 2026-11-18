# visao/rastreador_fino.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import sys
import time
import math
import threading
import numpy as np
from ultralytics import YOLO

from controle_voo.movimento import enviar_velocidade, enviar_posicao_xy_velocidade_z
from controle_voo.gimbal import apontar_gimbal_nadir

try:
    from gazebo_pose import ler_poses_absolutas_gazebo
except ImportError:
    _MISSAO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if _MISSAO_DIR not in sys.path:
        sys.path.insert(0, _MISSAO_DIR)
    from gazebo_pose import ler_poses_absolutas_gazebo

class RastreadorFinoYOLO(Node):
    def __init__(self, vehicle, logger=None, mostrar_janela=True):
        super().__init__('rastreador_fino_yolo')
        self.vehicle = vehicle
        self.logger = logger
        self.mostrar_janela = mostrar_janela
        self.bridge = CvBridge()
        
        # 1. Carrega o modelo YOLO da Fase 2 (Melhor treinado: YOLOv10n - mAP50-95 de 89.08% - 2 Classes: 0: alca, 1: armadilha)
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', '..', 'modelos_IA', 'YOLOv10n_Fase2_Armadilha_Alcas', 'weights', 'best.pt')
        self.get_logger().info(f"Carregando modelo YOLO FASE 2 de: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)

        # Modelo YOLO da Etapa Final (0.35m -> 0.20m): YOLOv10n_EtapaFinal_Alinhamento (mAP50-95: 99.50% - 1 Classe: 0: Alinhamento)
        caminho_modelo_alinhamento = os.path.join(
            os.path.dirname(__file__), '..', '..', 'modelos_IA',
            'YOLOv10n_EtapaFinal_Alinhamento', 'weights', 'best.pt'
        )
        self.get_logger().info(f"Carregando modelo YOLO ETAPA FINAL (0.20m) de: {caminho_modelo_alinhamento}")
        self.yolo_alinhamento = YOLO(caminho_modelo_alinhamento)
        
        # 2. Configurações de Controle (Ajuste Fino)
        # Ganhos bem menores que a Fase 1 para evitar oscilações brutas de perto
        self.kp = 0.15  
        self.kd = 0.05
        self.kp_alinhamento = 0.22
        self.kd_alinhamento = 0.08
        self.vel_max_alinhamento = 0.035
        
        # Ganhos para a rotação (Yaw)
        self.kp_yaw = 0.4
        self.kd_yaw = 0.1
        
        self.erro_x_anterior = 0.0
        self.erro_y_anterior = 0.0
        self.erro_yaw_anterior = 0.0
        self.ultimo_tempo = None
        
        # 3. Geometria Física e Máquina de Estados (5 Etapas)
        self.altura_gaiola = 0.317         # Altura exata da tampa da gaiola (sem alças, extraída do STL)
        self.altura_camera_offset = 0.320  # Distância dos pés de pouso do drone até a lente da câmera (m)
        
        # Proporção do Offset do Alvo para o modelo da Fase 2 (armadilha completa 0.2550m x 0.2550m):
        # X: Câmera perfeitamente centrada no eixo lateral (desvio < 0.6 mm) -> offset_pct_x = 0.0000
        # Y: Mount do Gimbal a +0.04216m do centro do drone / Bounding Box de 0.2550m -> offset_pct_y = 0.1653
        self.offset_pct_x = 0.0000
        self.offset_pct_y = 0.1653

        # Proporção do Offset do Alvo para o modelo Etapa Final (classe 'Alinhamento', aro central ~72mm x 72mm):
        # Na armadilha-final.stl, o aro circular fica em (-0.00022m, +0.04133m), quase colinear ao eixo óptico da
        # câmera (+0.00057m, +0.04216m) quando o drone está centrado sobre a armadilha (pos_drone == pos_armadilha).
        # Calibração empírica sobre as 400 imagens rotuladas do Dataset Etapa Final (20-35cm):
        #   cx - box_cx = +0.24mm / 71.65mm = +0.0034 | box_cy - cy = +0.88mm / 72.14mm = +0.0121
        self.offset_pct_x_alinhamento = 0.0034
        self.offset_pct_y_alinhamento = 0.0121
        
        # Parâmetros das Etapas da Fase 2 + Etapa Final
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
        self.alt_transicao_alinhamento = 0.35 # Etapa 3 -> Etapa 4: Troca para novo modelo sem esperar alinhamento a 0.35m
        self.alt_final = 0.20              # Etapa 4: Hover a 0.20m para ajuste fino com o novo modelo Alinhamento
        self.margem_erro_fov_baixa_alt = 0.040 # Pausa descida 0.35m->0.20m se erro > 40mm para proteger o FOV
        self.margem_erro_cravado = 0.005   # Margem reduzida para 5 mm (0.005m) de erro no offset (ajuste fino rigoroso)
        self.tempo_cravado_necessario = 2.5 # 2.5 segundos sustentados continuamente <= 5 mm a 0.20m (50 frames contínuos)
        self.vel_descida_cega = 0.80       # Descida firme (m/s, igual ao LAND_SPD_MS) para encurtar janela de deriva no Blind Drop
        self.altitude_captura = 0.025      # Altitude bruta (m) próxima ao toque antes de acionar LAND
        self.ultimo_log_headless = 0.0
        
        self.erro_final_x = None
        self.erro_final_y = None
        self.gz_pre_resultado = {}
        self.iniciou_blind_drop = False
        self.tempo_inicio_blind_drop = None
        self.ultimo_alt_blind_drop = None
        self.tempo_alt_estagnada = None
        self.diagnostico_thread = None
        self.blind_drop_pos_norte = None
        self.blind_drop_pos_leste = None
        self.blind_drop_down_inicio = None
        self.blind_drop_down_limite = None
        self.blind_drop_yaw_rad = None
        
        # Variáveis da Máquina de Estados de Descida
        self.rotacao_concluida = False
        self.usando_modelo_alinhamento = False
        self.hover_final_iniciado = False
        self._resetar_derivada = False
        self.tempo_yaw_ok = None
        self.tempo_cravado = None
        self.etapa_atual_nome = "ETAPA 1/5: APROXIMACAO INICIAL"
        
        # Filtro EMA para Bounding Box
        self.ema_x1 = None
        self.ema_y1 = None
        self.ema_x2 = None
        self.ema_y2 = None
        self.ema_alpha = 0.5 # 0.5 suaviza bem sem adicionar muito atraso
        self.ema_alpha_alinhamento = 0.55 # Suavização equilibrada com modelo Etapa Final a 0.20m-0.35m
        
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
        if self.mostrar_janela:
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

        # --- ETAPA 5: DESCIDA FINAL (BLIND DROP) E VERIFICAÇÃO DE CAPTURA ---
        if self.iniciou_blind_drop:
            self._processar_passo_blind_drop(altitude_bruta, altitude_relativa, tempo_atual)
            return

        # Comuta imediatamente para o novo modelo Alinhamento ao cruzar 0.35m (sem esperar alinhamento total a 0.35m)
        if self.rotacao_concluida and not self.usando_modelo_alinhamento and altitude_bruta <= self.alt_transicao_alinhamento:
            self.usando_modelo_alinhamento = True
            self.ema_x1 = self.ema_y1 = self.ema_x2 = self.ema_y2 = None
            self._resetar_derivada = True
            if self.usar_filtro:
                self.filtro.reset()
                # Reduz R em relação a 0.5 (curta distância), mantendo suavização firme contra oscilações
                self.filtro.kf.R = np.array([[0.005, 0.0], [0.0, 0.005]], dtype=float)
                self.filtro.kf.Q = np.array([
                    [0.0005, 0.0, 0.0, 0.0],
                    [0.0, 0.0005, 0.0, 0.0],
                    [0.0, 0.0, 0.01, 0.0],
                    [0.0, 0.0, 0.0, 0.01],
                ], dtype=float)
            self.get_logger().info(
                f"[+] TRANSICAO @ {altitude_bruta:.2f}m: Trocando para YOLOv10n_EtapaFinal_Alinhamento (alinhamento final a {self.alt_final:.2f}m)."
            )

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # --- PERCEPÇÃO VISUAL (Fase 2: 2 Classes | Etapa Final: 1 Classe 'Alinhamento') ---
        # ISOLAMENTO DE IMAGEM: O YOLO recebe um clone intocado (cópia limpa) da câmera.
        # Nenhuma alteração gráfica do OpenCV contamina a entrada do modelo.
        img_yolo = cv_image.copy()
        modelo_ativo = self.yolo_alinhamento if self.usando_modelo_alinhamento else self.yolo
        resultados = modelo_ativo.predict(img_yolo, verbose=False, conf=0.5)
        
        centro_x, centro_y = None, None
        box_cx, box_cy = None, None
        detectou_alvo = False
        alcas_pts = []
        erro_angular_deg = None
        yaw_rate = 0.0
        
        # 1. Desenha a mira central (Eixo óptico da Câmera / Centro do Drone)
        cv2.line(cv_image, (int(self.cx) - 25, int(self.cy)), (int(self.cx) + 25, int(self.cy)), (255, 255, 255), 2)
        cv2.line(cv_image, (int(self.cx), int(self.cy) - 25), (int(self.cx), int(self.cy) + 25), (255, 255, 255), 2)
        cv2.circle(cv_image, (int(self.cx), int(self.cy)), 4, (255, 255, 255), -1)
        cv2.putText(cv_image, "MIRA (Centro Camera)", (int(self.cx) + 12, int(self.cy) - 10), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        
        boxes = resultados[0].boxes
        if len(boxes) > 0:
            if self.usando_modelo_alinhamento:
                # Modelo Etapa Final: 1 classe (0: Alinhamento)
                boxes_alvo = [b for b in boxes if int(b.cls[0].item()) == 0]
                boxes_alca = []
                offset_x_ativo = self.offset_pct_x_alinhamento
                offset_y_ativo = self.offset_pct_y_alinhamento
                cor_bbox = (0, 255, 0)
                label_alvo_offset = "ALVO (ALINHAMENTO)"
            else:
                # Modelo Fase 2: 2 classes (0: alca, 1: armadilha)
                boxes_alvo = [b for b in boxes if int(b.cls[0].item()) == 1]
                boxes_alca = [b for b in boxes if int(b.cls[0].item()) == 0]
                offset_x_ativo = self.offset_pct_x
                offset_y_ativo = self.offset_pct_y
                cor_bbox = (255, 255, 0)
                label_alvo_offset = "ALVO (OFFSET)"
            
            # 2. Analisa o alvo principal (Armadilha ou Alinhamento) e calcula o Alvo Virtual com Offset
            if len(boxes_alvo) > 0:
                detectou_alvo = True
                best_alvo = max(boxes_alvo, key=lambda b: float(b.conf[0].item()))
                x1_raw, y1_raw, x2_raw, y2_raw = map(float, best_alvo.xyxy[0].cpu().numpy())
                
                # Filtro EMA para suavizar a bounding box do alvo
                alpha_ema = self.ema_alpha_alinhamento if self.usando_modelo_alinhamento else self.ema_alpha
                if self.ema_x1 is None:
                    self.ema_x1, self.ema_y1, self.ema_x2, self.ema_y2 = x1_raw, y1_raw, x2_raw, y2_raw
                else:
                    self.ema_x1 = (alpha_ema * x1_raw) + ((1.0 - alpha_ema) * self.ema_x1)
                    self.ema_y1 = (alpha_ema * y1_raw) + ((1.0 - alpha_ema) * self.ema_y1)
                    self.ema_x2 = (alpha_ema * x2_raw) + ((1.0 - alpha_ema) * self.ema_x2)
                    self.ema_y2 = (alpha_ema * y2_raw) + ((1.0 - alpha_ema) * self.ema_y2)
                    
                x1, y1, x2, y2 = int(self.ema_x1), int(self.ema_y1), int(self.ema_x2), int(self.ema_y2)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), cor_bbox, 2)
                
                box_w = self.ema_x2 - self.ema_x1
                box_h = self.ema_y2 - self.ema_y1
                box_cx = self.ema_x1 + box_w / 2.0
                box_cy = self.ema_y1 + box_h / 2.0
                cv2.circle(cv_image, (int(box_cx), int(box_cy)), 5, (255, 165, 0), -1)
                
                # Alvo Virtual com Offset Proporcional calibrado
                alvo_x = box_cx + (box_w * offset_x_ativo)
                alvo_y = box_cy - (box_h * offset_y_ativo)
                
                # O alinhamento no Alvo Virtual com Offset atua após a trava de Yaw (Etapa 3 em diante)
                if self.rotacao_concluida:
                    centro_x, centro_y = alvo_x, alvo_y
                    cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 7, (0, 0, 255), -1)
                    cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 13, (0, 0, 255), 2)
                    cv2.putText(cv_image, label_alvo_offset, (int(alvo_x) + 16, int(alvo_y)), 
                                cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                else:
                    # Durante Etapas 1 e 2: mira no centro geométrico da armadilha (sem exibir offset)
                    centro_x, centro_y = box_cx, box_cy
                    cv2.putText(cv_image, "CENTRO GAIOLA", (int(box_cx) + 10, int(box_cy)), 
                                cv2.FONT_HERSHEY_DUPLEX, 0.45, (255, 165, 0), 1, cv2.LINE_AA)
                            
            # 3. Analisa as Alças e Controla o Yaw (Apenas na Parte 1, antes de travar o Yaw)
            if not self.rotacao_concluida:
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
                        cv2.rectangle(cv_image, (b_coords[0], b_coords[1]), (b_coords[2], b_coords[3]), (0, 0, 255), 1)
                        cv2.putText(cv_image, "DUPLICADA", (b_coords[0], b_coords[1] - 4), 
                                    cv2.FONT_HERSHEY_DUPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
                
                # Seleciona apenas as 3 alças estritamente distintas de maior confiança
                for b, coords in alcas_distintas[:3]:
                    pcx = (coords[0] + coords[2]) / 2.0
                    pcy = (coords[1] + coords[3]) / 2.0
                    alcas_pts.append((pcx, pcy))
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
                    cv2.circle(cv_image, (int(apex_pt[0]), int(apex_pt[1])), 8, (255, 0, 255), -1)
                    cv2.circle(cv_image, (int(apex_pt[0]), int(apex_pt[1])), 13, (255, 0, 255), 2)
                    cv2.putText(cv_image, "EKF P0", (int(apex_pt[0]) + 15, int(apex_pt[1]) + 4), 
                                cv2.FONT_HERSHEY_DUPLEX, 0.40, (255, 0, 255), 1, cv2.LINE_AA)
                    
                    # Pontos da Base em ciano (Pontos Base 1 e 2 filtrados)
                    cv2.circle(cv_image, (int(b1_pt[0]), int(b1_pt[1])), 6, (255, 255, 0), -1)
                    cv2.circle(cv_image, (int(b1_pt[0]), int(b1_pt[1])), 11, (255, 255, 0), 2)
                    cv2.putText(cv_image, "EKF P1", (int(b1_pt[0]) - 55, int(b1_pt[1]) + 4), 
                                cv2.FONT_HERSHEY_DUPLEX, 0.40, (255, 255, 0), 1, cv2.LINE_AA)
                                
                    cv2.circle(cv_image, (int(b2_pt[0]), int(b2_pt[1])), 6, (255, 255, 0), -1)
                    cv2.circle(cv_image, (int(b2_pt[0]), int(b2_pt[1])), 11, (255, 255, 0), 2)
                    cv2.putText(cv_image, "EKF P2", (int(b2_pt[0]) + 15, int(b2_pt[1]) + 4), 
                                cv2.FONT_HERSHEY_DUPLEX, 0.40, (255, 255, 0), 1, cv2.LINE_AA)
                    
                    # Linhas do triângulo filtrado
                    cv2.line(cv_image, (int(b1_pt[0]), int(b1_pt[1])), (int(b2_pt[0]), int(b2_pt[1])), (255, 255, 0), 2)
                    cv2.line(cv_image, (int(apex_pt[0]), int(apex_pt[1])), (int(b1_pt[0]), int(b1_pt[1])), (0, 255, 0), 2)
                    cv2.line(cv_image, (int(apex_pt[0]), int(apex_pt[1])), (int(b2_pt[0]), int(b2_pt[1])), (0, 255, 0), 2)
                    
                    # Seta de orientação calculada pelo ângulo filtrado
                    cv2.arrowedLine(cv_image, (int(mid_base[0]), int(mid_base[1])), 
                                    (int(apex_pt[0]), int(apex_pt[1])), (0, 255, 255), 2, tipLength=0.25)
                    cv2.putText(cv_image, "ALCAS FILTRADAS (EKF)", (int(mid_base[0]) - 85, int(mid_base[1]) - 14), 
                                cv2.FONT_HERSHEY_DUPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
                                
                    # Controle de Yaw (Ativo estritamente na Etapa 2: altitude_bruta <= 1.50m)
                    if altitude_bruta <= self.alt_yaw:
                        # ETAPA 2: Ajuste e trava de Yaw a 1.50m
                        if abs(erro_angular_deg) <= self.margem_erro_yaw_deg:
                            if self.tempo_yaw_ok is None:
                                self.tempo_yaw_ok = agora
                            elif (agora - self.tempo_yaw_ok) >= self.tempo_yaw_ok_necessario:
                                self.rotacao_concluida = True
                                self.ultimo_erro_yaw_deg = erro_angular_deg
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
                        # ETAPA 1: Acima de 1.50m o Yaw não gira
                        yaw_rate = 0.0
                        self.tempo_yaw_ok = None
                else:
                    # Menos de 3 alças encontradas
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
                        
                    if self.usar_filtro:
                        self.filtro_p0.atualizar(None, None, dt)
                        self.filtro_p1.atualizar(None, None, dt)
                        self.filtro_p2.atualizar(None, None, dt)
                        self.filtro_angulo.atualizar(None, dt)
            else:
                # Parte 2 (Yaw travado): não processa nem desenha bounding boxes das alças
                yaw_rate = 0.0
                self.tempo_sem_3_alcas = None
                    
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
            
            # Evita salto derivativo no primeiro frame após trocar o modelo YOLO
            if self._resetar_derivada:
                self.erro_x_anterior = erro_x
                self.erro_y_anterior = erro_y
                self._resetar_derivada = False

            # Derivadas de velocidade translacional
            derivada_x = (erro_x - self.erro_x_anterior) / dt
            derivada_y = (erro_y - self.erro_y_anterior) / dt
            self.erro_x_anterior = erro_x
            self.erro_y_anterior = erro_y
            
            erro_xy_atual = (erro_x**2 + erro_y**2)**0.5
            erro_mm = erro_xy_atual * 1000.0
            
            # --- MÁQUINA DE ESTADOS DA FASE 2 + ETAPA FINAL (5 ETAPAS) ---
            kd_atual = self.kd
            if not self.rotacao_concluida:
                # ETAPAS ANTERIORES À TRAVA DE YAW (Centralizado na Armadilha)
                if altitude_bruta > self.alt_yaw:
                    # ETAPA 1: Aproximação inicial até 1.50m
                    vel_z = 0.20
                    yaw_rate = 0.0 # Yaw travado, não gira ainda
                    self.tempo_yaw_ok = None
                    self.tempo_cravado = None
                    max_vel = 0.4
                    kp_atual = self.kp
                    self.etapa_atual_nome = f"ETAPA 1/5: APROXIMACAO INICIAL (Descendo para {self.alt_yaw:.2f}m)"
                    cor_status = (0, 165, 255) # Laranja
                else:
                    # ETAPA 2: Hover a 1.50m para alinhamento e trava de Yaw com Kalman nas alças
                    vel_z = 0.0 # Hover estável a 1.50m
                    self.tempo_cravado = None
                    max_vel = 0.25
                    kp_atual = self.kp
                    
                    if erro_angular_deg is not None:
                        if self.tempo_yaw_ok is not None:
                            tempo_yaw_acumulado = agora - self.tempo_yaw_ok
                            self.etapa_atual_nome = f"ETAPA 2/5: AJUSTE YAW @ {self.alt_yaw:.2f}m | Erro: {erro_angular_deg:+.1f}° ({tempo_yaw_acumulado:.2f}s/{self.tempo_yaw_ok_necessario:.2f}s)"
                            cor_status = (0, 255, 0) # Verde se estiver sustentando dentro da meta
                        else:
                            self.etapa_atual_nome = f"ETAPA 2/5: AJUSTE YAW @ {self.alt_yaw:.2f}m | Erro: {erro_angular_deg:+.1f}° (Meta <={self.margem_erro_yaw_deg:.1f}°)"
                            cor_status = (0, 255, 255) # Amarelo
                    else:
                        self.tempo_yaw_ok = None
                        tempo_sem = (agora - self.tempo_sem_3_alcas) if self.tempo_sem_3_alcas is not None else 0.0
                        if tempo_sem >= self.timeout_sombra_alca:
                            self.etapa_atual_nome = f"ETAPA 2/5: DESVIO DE SOMBRA! Forcando Rotacao ({len(alcas_pts)}/3 alcas)"
                            cor_status = (255, 0, 255) # Magenta para indicar desvio de sombra
                        else:
                            self.etapa_atual_nome = f"ETAPA 2/5: BUSCANDO 3 ALCAS ({len(alcas_pts)}/3) - {tempo_sem:.1f}s/{self.timeout_sombra_alca:.1f}s"
                            cor_status = (0, 165, 255) # Laranja
            else:
                # ETAPAS APÓS TRAVA DE YAW (Centralizado no Alvo Virtual com Offset)
                yaw_rate = 0.0 # Yaw permanentemente travado

                if self.hover_final_iniciado and altitude_bruta > self.alt_final + 0.04:
                    self.hover_final_iniciado = False

                if not self.usando_modelo_alinhamento:
                    # ETAPA 3: Descida no Offset (Fase 2) até 0.35m (sem parar para alinhamento total a 0.35m)
                    dist_descida = altitude_bruta - self.alt_final
                    fator_frenagem = min(1.0, max(0.25, dist_descida / 0.25))
                    vel_z = 0.18 * fator_frenagem
                    self.tempo_cravado = None
                    max_vel = 0.35
                    kp_atual = self.kp
                    self.etapa_atual_nome = f"ETAPA 3/5: DESCENDO NO OFFSET (Troca @ {self.alt_transicao_alinhamento:.2f}m)"
                    cor_status = (0, 220, 0) # Verde
                elif (not self.hover_final_iniciado) and altitude_bruta > self.alt_final:
                    # ETAPA 4A: Descida de 0.35m até 0.20m guiada pelo novo modelo Alinhamento
                    self.tempo_cravado = None
                    max_vel = 0.25
                    kp_atual = self.kp
                    if (not detectou_alvo) or erro_xy_atual > self.margem_erro_fov_baixa_alt:
                        vel_z = 0.0
                        self.etapa_atual_nome = f"ETAPA 4/5: CENTRALIZANDO ALINHAMENTO ({erro_mm:.1f}mm)"
                        cor_status = (0, 165, 255)
                    else:
                        dist_descida = altitude_bruta - self.alt_final
                        # Desacelera suavemente nos últimos 25cm para pousar o hover em 0.20m sem overshoot
                        fator_frenagem = min(1.0, max(0.25, dist_descida / 0.25))
                        vel_z = 0.18 * fator_frenagem
                        self.etapa_atual_nome = f"ETAPA 4/5: DESCENDO P/ {self.alt_final:.2f}m (Modelo Alinhamento)"
                        cor_status = (0, 220, 0) # Verde
                else:
                    # ETAPA 4B: Ajuste Fino a 0.20m com novo modelo Alinhamento (margem estrita <= 5 mm por 2.5s)
                    self.hover_final_iniciado = True
                    vel_z = 0.0 # Hover a 0.20m
                    max_vel = self.vel_max_alinhamento
                    kp_atual = self.kp_alinhamento
                    kd_atual = self.kd_alinhamento
                    
                    # Critério estrito: Alinhamento cravado <= 5 mm sustentado continuamente por 2.5s apenas após atingir 0.20m
                    if detectou_alvo and erro_xy_atual <= self.margem_erro_cravado:
                        if self.tempo_cravado is None:
                            self.tempo_cravado = agora
                        tempo_acumulado = agora - self.tempo_cravado
                        meta_label = f"<={self.margem_erro_cravado*1000:.0f}mm"
                        self.etapa_atual_nome = f"ETAPA 4/5: AJUSTE FINO @ {self.alt_final:.2f}m {meta_label} ({erro_mm:.1f}mm) - {tempo_acumulado:.1f}s/{self.tempo_cravado_necessario:.1f}s"
                        cor_status = (0, 255, 0) # Verde brilhante
                        
                        if tempo_acumulado >= self.tempo_cravado_necessario:
                            self._configurar_trava_blind_drop(erro_x, erro_y, altitude_bruta)
                            self.iniciou_blind_drop = True
                            self.tempo_inicio_blind_drop = agora
                            self.ultimo_alt_blind_drop = altitude_bruta
                            self.tempo_alt_estagnada = None
                            if self.mostrar_janela:
                                try:
                                    cv2.destroyWindow(self.nome_janela)
                                except Exception:
                                    pass
                            self._enviar_comando_blind_drop(0.0)
                            self.diagnostico_thread = threading.Thread(
                                target=self._imprimir_diagnostico_pre_blind_drop,
                                args=(erro_x, erro_y, erro_mm, box_cx, box_cy, altitude_relativa),
                                daemon=True,
                            )
                            self.diagnostico_thread.start()
                            self.get_logger().info(f"[+] ETAPA 4 CONCLUIDA [PRECISAO CRAVADA @ {self.alt_final:.2f}m]: Alvo a {erro_mm:.2f} mm! Disparando Etapa 5 (Blind Drop travado XY a {self.vel_descida_cega} m/s).")
                            return
                    else:
                        self.tempo_cravado = None
                        self.etapa_atual_nome = f"ETAPA 4/5: AJUSTE FINO @ {self.alt_final:.2f}m | Erro: {erro_mm:.1f}mm (Meta <={self.margem_erro_cravado*1000:.0f}mm)"
                        cor_status = (0, 255, 255) # Amarelo alerta
                        
            vel_x = (erro_x * kp_atual + derivada_x * kd_atual)
            vel_y = (erro_y * kp_atual + derivada_y * kd_atual)
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # Envia velocidade translacional e taxa de Yaw para o drone
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z, yaw_rate)
            if self.logger:
                self.logger.registrar(
                    fase='FASE 2: MICRO',
                    etapa=self.etapa_atual_nome,
                    altitude=altitude_bruta,
                    erro_x=erro_x,
                    erro_y=erro_y,
                    erro_yaw_deg=(self.ultimo_erro_yaw_deg if self.rotacao_concluida else erro_angular_deg),
                    vel_x=vel_x,
                    vel_y=vel_y,
                    vel_z=vel_z,
                    yaw_rate=yaw_rate,
                    afastamento_fov_pct=0.0,
                    descendo=(vel_z > 0.0),
                    bloqueios=0,
                    ekf_ativo=self.usar_filtro
                )
        else:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
            if self.logger:
                self.logger.registrar(
                    fase='FASE 2: MICRO',
                    etapa=self.etapa_atual_nome,
                    altitude=altitude_bruta,
                    erro_x=None,
                    erro_y=None,
                    erro_yaw_deg=None,
                    vel_x=0.0,
                    vel_y=0.0,
                    vel_z=0.0,
                    yaw_rate=0.0,
                    afastamento_fov_pct=0.0,
                    descendo=False,
                    bloqueios=0,
                    ekf_ativo=False
                )
            self.etapa_atual_nome = "ALVO PERDIDO (Aguardando)"
            cor_status = (0, 0, 255)
            self.tempo_yaw_ok = None
            self.tempo_cravado = None
            
        if not self.rotacao_concluida and erro_angular_deg is not None:
            self.ultimo_erro_yaw_deg = erro_angular_deg

        if not self.mostrar_janela:
            if (agora - self.ultimo_log_headless) >= 3.0:
                self.ultimo_log_headless = agora
                info_erro = f" | Erro={erro_mm:.1f}mm" if 'erro_xy_atual' in locals() else ""
                self.get_logger().info(f"[HEADLESS] Alt={altitude_bruta:.2f}m{info_erro} | {self.etapa_atual_nome}")
            return

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
        
        # Formatação do Status e Erro de Yaw (travado após conclusão da Etapa 2)
        status_yaw_str = "TRAVADO" if self.rotacao_concluida else "ALINHANDO"
        if self.rotacao_concluida and self.ultimo_erro_yaw_deg is not None:
            erro_yaw_str = f"{self.ultimo_erro_yaw_deg:+.1f}° (Travado)"
        elif erro_angular_deg is not None:
            self.ultimo_erro_yaw_deg = erro_angular_deg
            erro_yaw_str = f"{erro_angular_deg:+.1f}°"
        else:
            erro_yaw_str = "N/A"
        
        # Textos do Painel de Status
        cv2.putText(cv_image, f"Alt Bruta: {altitude_bruta:.2f}m | Alt Rel: {altitude_relativa:.2f}m | Yaw: {status_yaw_str} (Erro: {erro_yaw_str})", 
                    (28, 38), cv2.FONT_HERSHEY_DUPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(cv_image, self.etapa_atual_nome, (28, 72), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.65, cor_status, 1, cv2.LINE_AA)
                    
        if 'erro_xy_atual' in locals():
            if self.usando_modelo_alinhamento:
                label_alvo = "Erro Alinhamento (0.20m)"
            elif self.rotacao_concluida:
                label_alvo = "Erro Offset"
            else:
                label_alvo = "Erro Centro Gaiola"
            cor_erro = (0, 255, 0) if (self.rotacao_concluida and erro_xy_atual <= self.margem_erro_cravado) else (0, 255, 255)
            cv2.putText(cv_image, f"{label_alvo}: {erro_mm:.1f} mm  |  Meta Fina: <= {self.margem_erro_cravado*1000:.1f} mm ({self.tempo_cravado_necessario:.1f}s)", 
                        (28, 102), cv2.FONT_HERSHEY_DUPLEX, 0.55, cor_erro, 1, cv2.LINE_AA)
                        
        # Textos da Caixa de Legendas (Enxuta e com indicação clara do erro em graus)
        cv2.putText(cv_image, "=== LEGENDA & ORIENTACAO ===", (leg_x1 + 80, leg_y1 + 25), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(cv_image, "[+] Mira Drone   (*) Alvo Offset   [^] Alcas EKF", (leg_x1 + 15, leg_y1 + 53), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.40, (200, 200, 200), 1, cv2.LINE_AA)
        
        # Linha destacada mostrando quantos graus está errado
        if self.rotacao_concluida and self.ultimo_erro_yaw_deg is not None:
            cor_yaw_leg = (0, 255, 0)
            texto_yaw_leg = f"Erro de Yaw: {self.ultimo_erro_yaw_deg:+.1f}° (TRAVADO)"
        elif erro_angular_deg is not None:
            cor_yaw_leg = (0, 255, 0) if abs(erro_angular_deg) <= self.margem_erro_yaw_deg else (0, 255, 255)
            texto_yaw_leg = f"Erro de Yaw: {erro_angular_deg:+.1f}° (Meta <= {self.margem_erro_yaw_deg:.1f}°)"
        elif self.tempo_sem_3_alcas is not None and (agora - self.tempo_sem_3_alcas) >= self.timeout_sombra_alca:
            cor_yaw_leg = (255, 0, 255)
            texto_yaw_leg = f"Yaw: DESVIANDO SOMBRA ({len(alcas_pts)}/3)"
        else:
            cor_yaw_leg = (0, 165, 255)
            texto_yaw_leg = f"Erro de Yaw: Buscando alcas ({len(alcas_pts)}/3)..."
            
        cv2.putText(cv_image, texto_yaw_leg, (leg_x1 + 20, leg_y1 + 85), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.50, cor_yaw_leg, 1, cv2.LINE_AA)
                    
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)

    def _configurar_trava_blind_drop(self, erro_x, erro_y, altitude_bruta):
        """Trava a coordenada horizontal NED exata do alvo (drone + resíduo óptico rotacionado) antes da descida cega."""
        loc = getattr(getattr(self.vehicle, 'location', None), 'local_frame', None)
        yaw_atual = getattr(getattr(self.vehicle, 'attitude', None), 'yaw', None)
        if (
            loc is not None
            and loc.north is not None
            and loc.east is not None
            and loc.down is not None
            and yaw_atual is not None
        ):
            cos_y = math.cos(yaw_atual)
            sin_y = math.sin(yaw_atual)
            delta_n = (erro_x * cos_y) - (erro_y * sin_y)
            delta_e = (erro_x * sin_y) + (erro_y * cos_y)
            self.blind_drop_pos_norte = float(loc.north) + delta_n
            self.blind_drop_pos_leste = float(loc.east) + delta_e
            self.blind_drop_down_inicio = float(loc.down)
            self.blind_drop_down_limite = float(loc.down) + max(altitude_bruta, self.alt_final) + 0.15
            self.blind_drop_yaw_rad = float(yaw_atual)
        else:
            self.blind_drop_pos_norte = None
            self.blind_drop_pos_leste = None
            self.blind_drop_down_inicio = None
            self.blind_drop_down_limite = None
            self.blind_drop_yaw_rad = None

    def _enviar_comando_blind_drop(self, dt_blind, finalizar=False):
        """Mantém trava XY em LOCAL_NED via PosVelAccel durante o Blind Drop (ou fallback em velocidade zero XY)."""
        if self.blind_drop_pos_norte is not None:
            if finalizar:
                baixo_z = self.blind_drop_down_limite
                vel_z_cmd = 0.0
            else:
                baixo_z = min(
                    self.blind_drop_down_inicio + (self.vel_descida_cega * dt_blind),
                    self.blind_drop_down_limite,
                )
                vel_z_cmd = self.vel_descida_cega if baixo_z < self.blind_drop_down_limite else 0.0
            enviar_posicao_xy_velocidade_z(
                self.vehicle,
                self.blind_drop_pos_norte,
                self.blind_drop_pos_leste,
                baixo_z,
                vel_z_cmd,
                yaw_rad=self.blind_drop_yaw_rad,
            )
            return vel_z_cmd

        vel_z_cmd = 0.0 if finalizar else self.vel_descida_cega
        enviar_velocidade(self.vehicle, 0.0, 0.0, vel_z_cmd, 0.0)
        return vel_z_cmd

    def _processar_passo_blind_drop(self, altitude_bruta, altitude_relativa, agora):
        """Comanda a descida vertical travada no plano horizontal e detecta o toque próximo ao alvo."""
        if self.tempo_inicio_blind_drop is None:
            self.tempo_inicio_blind_drop = agora
        dt_blind = agora - self.tempo_inicio_blind_drop

        if self.ultimo_alt_blind_drop is not None and abs(altitude_bruta - self.ultimo_alt_blind_drop) < 0.004:
            if self.tempo_alt_estagnada is None:
                self.tempo_alt_estagnada = agora
        else:
            self.tempo_alt_estagnada = None
            self.ultimo_alt_blind_drop = altitude_bruta

        tocou_chao = (altitude_bruta <= self.altitude_captura or altitude_relativa <= self.altitude_captura)
        estagnado_no_alvo = (
            dt_blind >= 0.7
            and altitude_bruta <= 0.06
            and self.tempo_alt_estagnada is not None
            and (agora - self.tempo_alt_estagnada) >= 0.6
        )
        timeout_blind = (dt_blind >= 6.0)

        if tocou_chao or estagnado_no_alvo or timeout_blind:
            self._enviar_comando_blind_drop(dt_blind, finalizar=True)
            self.get_logger().info("CAPTURA BEM SUCEDIDA! Altura de toque atingida.")

            if self.erro_final_x is not None:
                rmse = (self.erro_final_x**2 + self.erro_final_y**2)**0.5
                self.get_logger().info("========== RESULTADO PARA O TCC ==========")
                self.get_logger().info(f"ERRO FINAL (RMSE) NO PONTO DE CORTE: {rmse:.4f} metros ({rmse*1000.0:.1f} mm)")
                self.get_logger().info("==========================================")

            raise SystemExit

        vel_z = self._enviar_comando_blind_drop(dt_blind)
        if self.logger:
            self.logger.registrar(
                fase='FASE 2: MICRO',
                etapa='ETAPA 5: BLIND DROP',
                altitude=altitude_bruta,
                erro_x=self.erro_final_x,
                erro_y=self.erro_final_y,
                erro_yaw_deg=self.ultimo_erro_yaw_deg,
                vel_x=0.0,
                vel_y=0.0,
                vel_z=vel_z,
                yaw_rate=0.0,
                afastamento_fov_pct=0.0,
                descendo=True,
                bloqueios=0,
                ekf_ativo=False
            )

    def _imprimir_diagnostico_pre_blind_drop(self, erro_x_m, erro_y_m, erro_mm, box_cx, box_cy, altitude_relativa):
        """Imprime e compara a distância real XY no Gazebo e o erro medido pela câmera antes do Blind Drop."""
        erro_x_mm = erro_x_m * 1000.0
        erro_y_mm = erro_y_m * 1000.0

        dist_bruta_lente_mm = None
        if box_cx is not None and box_cy is not None:
            dx_lente_m = (box_cx - self.cx) * altitude_relativa / self.fx
            dy_lente_m = (box_cy - self.cy) * altitude_relativa / self.fy
            dist_bruta_lente_mm = math.hypot(dx_lente_m, dy_lente_m) * 1000.0

        poses_mundo = ler_poses_absolutas_gazebo(world_name='iris_runway', tentativas=2, timeout_s=2.0)

        print("\n" + "=" * 68, flush=True)
        print("🔎 VERIFICAÇÃO PRÉ-BLIND DROP (Centro Drone <-> Centro Armadilha, XY)", flush=True)
        if poses_mundo is not None:
            pos_drone = poses_mundo['drone_coaxial']['position']
            pos_armadilha = poses_mundo['armadilha_alvo']['position']
            dx_mundo_mm = (pos_drone['x'] - pos_armadilha['x']) * 1000.0
            dy_mundo_mm = (pos_drone['y'] - pos_armadilha['y']) * 1000.0
            dist_mundo_mm = math.hypot(dx_mundo_mm, dy_mundo_mm)
            diferenca_mm = abs(dist_mundo_mm - erro_mm)
            self.gz_pre_resultado.update({
                'dx_mm': round(dx_mundo_mm, 2),
                'dy_mm': round(dy_mundo_mm, 2),
                'rmse_mm': round(dist_mundo_mm, 2),
                'diff_mm': round(diferenca_mm, 2),
                'lente_bruta_mm': round(dist_bruta_lente_mm, 2) if dist_bruta_lente_mm is not None else None,
            })
            print(f"   * Distância Real (Gazebo Mundo) : {dist_mundo_mm:.2f} mm (dX={dx_mundo_mm:+.2f} mm, dY={dy_mundo_mm:+.2f} mm)", flush=True)
            print(f"   * Drone Mundo (X, Y)            : ({pos_drone['x']:.4f} m, {pos_drone['y']:.4f} m)", flush=True)
            print(f"   * Armadilha Mundo (X, Y)        : ({pos_armadilha['x']:.4f} m, {pos_armadilha['y']:.4f} m)", flush=True)
        else:
            diferenca_mm = None
            print("   * Distância Real (Gazebo Mundo) : [indisponível - falha ao ler /world/iris_runway/pose/info]", flush=True)

        print(f"   * Erro Medido Câmera (EKF/Alvo) : {erro_mm:.2f} mm (eX={erro_x_mm:+.2f} mm, eY={erro_y_mm:+.2f} mm)", flush=True)
        if dist_bruta_lente_mm is not None:
            print(f"   * Dist. Bruta Lente<->Gaiola    : {dist_bruta_lente_mm:.2f} mm (sem compensação de offset)", flush=True)
        if diferenca_mm is not None:
            print(f"   * Diferença |Real - Câmera|     : {diferenca_mm:.2f} mm", flush=True)
        print("=" * 68 + "\n", flush=True)

