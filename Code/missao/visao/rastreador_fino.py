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
        
        # 1. Carrega o modelo YOLO da Fase 2 (2 Classes: 0: alca, 1: armadilha)
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', '..', 'modelos_IA', 'YOLOv11n_Fase2_Armadilha_Alcas', 'weights', 'best.pt')
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
        
        # 3. Geometria Física e Máquina de Estados (Passo 4 e 5)
        self.altura_gaiola = 0.36 # Altura exata extraída do STL
        
        # Proporção do Offset do Alvo (Em relação ao tamanho da Bounding Box)
        # Valores calculados pela ferramenta de calibração visual
        self.offset_pct_x = -0.0066
        self.offset_pct_y = 0.1542
        
        self.altitude_captura = 0.05 # Altitude RELATIVA que consideramos "Toque" na gaiola (Captura)
        
        self.erro_final_x = None
        self.erro_final_y = None
        self.iniciou_blind_drop = False
        
        # Variáveis da Máquina de Estados de Descida
        self.rotacao_concluida = False
        self.tempo_yaw_ok = None
        self.tempo_alinhamento_ok = None
        self.tempo_borda_ok = None
        self.tempo_cravado = None
        
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

    def image_callback(self, msg):
        tempo_atual = time.time()
        if self.ultimo_tempo is not None:
            if (tempo_atual - self.ultimo_tempo) < 0.05:
                return # Limita a ~20 FPS (Exigência física/visual do sistema)
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
            
            # Desliga a câmera (fecha a janela) simulando perda de feed de vídeo
            try:
                cv2.destroyWindow(self.nome_janela)
            except:
                pass
                
            return

        # --- PASSO 3: FUNIL VISUAL (Visão Ativa - YOLO 2 Classes) ---
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.5)
        
        centro_x, centro_y = None, None
        alcas_pts = []
        erro_angular_deg = None
        yaw_rate = 0.0
        
        # Desenha a mira (Centro da tela)
        cv2.line(cv_image, (int(self.cx) - 20, int(self.cy)), (int(self.cx) + 20, int(self.cy)), (255, 255, 255), 2)
        cv2.line(cv_image, (int(self.cx), int(self.cy) - 20), (int(self.cx), int(self.cy) + 20), (255, 255, 255), 2)
        
        img_h, img_w = cv_image.shape[:2]
        tocar_borda = (altitude_bruta <= 0.75)
        
        boxes = resultados[0].boxes
        if len(boxes) > 0:
            # Separa detecções por classe (0: alca, 1: armadilha)
            boxes_armadilha = [b for b in boxes if int(b.cls[0].item()) == 1]
            boxes_alca = [b for b in boxes if int(b.cls[0].item()) == 0]
            
            # 1. Analisa a Armadilha (Gaiola inteira) e calcula o Alvo Virtual com Offset
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
                cv2.circle(cv_image, (int(box_cx), int(box_cy)), 4, (255, 255, 0), -1)
                
                # Alvo Virtual com Offset Proporcional
                # offset_pct_x positivo = Direita, offset_pct_y positivo = Cima (Y negativo no pixel)
                alvo_x = box_cx + (box_w * self.offset_pct_x)
                alvo_y = box_cy - (box_h * self.offset_pct_y)
                centro_x, centro_y = alvo_x, alvo_y
                
                cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 8, (0, 0, 255), -1)
                cv2.putText(cv_image, "ALVO VIRTUAL", (int(alvo_x)+10, int(alvo_y)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            
            # 2. Analisa as Alças para Cálculo de Ângulo (Triângulo Isósceles)
            sorted_alcas = sorted(boxes_alca, key=lambda b: float(b.conf[0].item()), reverse=True)[:3]
            for b in sorted_alcas:
                bx1, by1, bx2, by2 = map(int, b.xyxy[0].cpu().numpy())
                pcx = (bx1 + bx2) / 2.0
                pcy = (by1 + by2) / 2.0
                alcas_pts.append((pcx, pcy))
                cv2.rectangle(cv_image, (bx1, by1), (bx2, by2), (0, 220, 255), 1)
                cv2.circle(cv_image, (int(pcx), int(pcy)), 4, (0, 220, 255), -1)
                
            agora = time.time()
            
            # Requisito: O drone SÓ pode rotacionar quando de fato estiver vendo as 3 alças
            if len(alcas_pts) == 3:
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
                # Produto vetorial 2D
                cross = u_raw[0] * v_cand[1] - u_raw[1] * v_cand[0]
                if cross > 0:
                    b1_raw, b2_raw = cand_b1, cand_b2
                else:
                    b1_raw, b2_raw = cand_b2, cand_b1
                    
                # Aplica o Filtro de Kalman (FiltroAlvoEKF) nos 3 pontos individualmente
                if self.usar_filtro:
                    # Converte pontos para metros físicos relativos à câmera
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
                        # Converte de volta para pixel para cálculos e HUD
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
                    
                # Ponto médio da base do triângulo filtrado
                mid_base = (b1_pt + b2_pt) / 2.0
                
                # Vetor do meio da base apontando para o vértice
                vec = apex_pt - mid_base
                dx, dy = float(vec[0]), float(vec[1])
                angulo_atual_bruto = math.atan2(dy, dx)
                
                if self.usar_filtro:
                    angulo_atual = self.filtro_angulo.atualizar(angulo_atual_bruto, dt)
                    if angulo_atual is None:
                        angulo_atual = angulo_atual_bruto
                else:
                    angulo_atual = angulo_atual_bruto
                    
                # Requisito: Base para cima e vértice para baixo
                # No OpenCV, apontar para baixo (Y positivo) é +PI/2 radianos (+90 graus)
                angulo_alvo = math.pi / 2.0
                erro_angular = (angulo_atual - angulo_alvo + math.pi) % (2.0 * math.pi) - math.pi
                erro_angular_deg = math.degrees(erro_angular)
                
                # Desenho do triângulo isósceles filtrado no HUD
                # Base em ciano
                cv2.line(cv_image, (int(b1_pt[0]), int(b1_pt[1])), (int(b2_pt[0]), int(b2_pt[1])), (255, 255, 0), 2)
                # Lados similares em verde
                cv2.line(cv_image, (int(apex_pt[0]), int(apex_pt[1])), (int(b1_pt[0]), int(b1_pt[1])), (0, 255, 0), 2)
                cv2.line(cv_image, (int(apex_pt[0]), int(apex_pt[1])), (int(b2_pt[0]), int(b2_pt[1])), (0, 255, 0), 2)
                cv2.circle(cv_image, (int(mid_base[0]), int(mid_base[1])), 4, (255, 255, 0), -1)
                # Vértice em magenta
                cv2.circle(cv_image, (int(apex_pt[0]), int(apex_pt[1])), 7, (255, 0, 255), -1)
                cv2.putText(cv_image, "VERTICE", (int(apex_pt[0])+10, int(apex_pt[1])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
                # Vetor de orientação (meio da base -> vértice)
                cv2.arrowedLine(cv_image, (int(mid_base[0]), int(mid_base[1])), 
                                (int(apex_pt[0]), int(apex_pt[1])), (0, 255, 255), 2, tipLength=0.25)
                                
                # Requisito: O ajuste de rotação e alinhamento fino ocorre no Ajuste Final (<= 0.75m)
                if tocar_borda:
                    if not self.rotacao_concluida:
                        # Variação máxima de 2° mantendo por 0.5 segundos contínuos
                        if abs(erro_angular_deg) <= 2.0:
                            if self.tempo_yaw_ok is None:
                                self.tempo_yaw_ok = agora
                            elif (agora - self.tempo_yaw_ok) >= 0.5:
                                self.rotacao_concluida = True
                                self.get_logger().info(f"[+] AJUSTE FINAL: YAW ALINHADO E TRAVADO! Erro: {erro_angular_deg:.2f}° mantido por 0.5s.")
                        else:
                            self.tempo_yaw_ok = None
                            
                        derivada_yaw = (erro_angular - self.erro_yaw_anterior) / dt
                        self.erro_yaw_anterior = erro_angular
                        yaw_rate = erro_angular * self.kp_yaw + derivada_yaw * self.kd_yaw
                        max_yaw = 0.5
                        yaw_rate = max(-max_yaw, min(max_yaw, yaw_rate))
                    else:
                        # Yaw travado permanentemente
                        yaw_rate = 0.0
                else:
                    # Durante a descida inicial (altitude > 0.75m), o drone NÃO rotaciona (foca apenas no offset)
                    yaw_rate = 0.0
                    self.tempo_yaw_ok = None
            else:
                # O drone NÃO rotaciona se não estiver vendo as 3 alças
                yaw_rate = 0.0
                if not self.rotacao_concluida:
                    self.tempo_yaw_ok = None
                if self.usar_filtro:
                    self.filtro_p0.atualizar(None, None, dt)
                    self.filtro_p1.atualizar(None, None, dt)
                    self.filtro_p2.atualizar(None, None, dt)
                    self.filtro_angulo.atualizar(None, dt)
                    
        # Converte o Alvo Virtual de pixel para metros físicos (PBVS)
        x_m_bruto, y_m_bruto = None, None
        if centro_x is not None and centro_y is not None:
            x_m_bruto = (centro_x - self.cx) * altitude_relativa / self.fx
            y_m_bruto = (centro_y - self.cy) * altitude_relativa / self.fy
            
        # Filtro EKF (Lidando com Jitter)
        if self.usar_filtro:
            erro_x, erro_y = self.filtro.atualizar(x_m_bruto, y_m_bruto, dt)
            if erro_x is not None and erro_y is not None:
                pixel_x_filtrado = int((erro_x * self.fx / altitude_relativa) + self.cx)
                pixel_y_filtrado = int((erro_y * self.fy / altitude_relativa) + self.cy)
                cv2.circle(cv_image, (pixel_x_filtrado, pixel_y_filtrado), 6, (255, 255, 0), -1)
                cv2.putText(cv_image, "FILTRADO", (pixel_x_filtrado+10, pixel_y_filtrado), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        else:
            erro_x, erro_y = x_m_bruto, y_m_bruto
            
        if erro_x is not None and erro_y is not None:
            self.erro_final_x = erro_x
            self.erro_final_y = erro_y
            
            # PD de Translação (X e Y) centralizando no Offset
            derivada_x = (erro_x - self.erro_x_anterior) / dt
            derivada_y = (erro_y - self.erro_y_anterior) / dt
            self.erro_x_anterior = erro_x
            self.erro_y_anterior = erro_y
            
            if tocar_borda:
                max_vel = 0.02
                kp_atual = 0.15
            else:
                max_vel = 0.5
                kp_atual = self.kp
                
            vel_x = (erro_x * kp_atual + derivada_x * self.kd)
            vel_y = (erro_y * kp_atual + derivada_y * self.kd)
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            erro_xy_atual = (erro_x**2 + erro_y**2)**0.5
            agora = time.time()
            
            if tocar_borda:
                # MOMENTO 2: Ajuste Final de Rotação e Alinhamento (<= 0.75m)
                if self.tempo_borda_ok is None:
                    self.tempo_borda_ok = agora
                    
                tempo_restante = max(0.0, 10.0 - (agora - self.tempo_borda_ok))
                segundos_arredondados = int(math.ceil(tempo_restante))
                if not hasattr(self, 'ultimo_seg_impresso') or self.ultimo_seg_impresso != segundos_arredondados:
                    if segundos_arredondados > 0:
                        self.get_logger().info(f"[!] AJUSTE FINAL (YAW + OFFSET): {segundos_arredondados}s restantes...")
                    self.ultimo_seg_impresso = segundos_arredondados
                    
                if tempo_restante <= 0.0:
                    # Só dispara a queda cega se o Yaw já estiver travado E o offset estiver cravado (<0.1cm por 0.5s)
                    if not self.rotacao_concluida:
                        self.tempo_cravado = None
                        if erro_angular_deg is not None:
                            texto_status = f"AJUSTE FINAL: ALINHANDO YAW (Erro: {erro_angular_deg:.1f}° > 2°)"
                        else:
                            texto_status = "AJUSTE FINAL: BUSCANDO 3 ALÇAS P/ YAW"
                        cor_status = (0, 165, 255)
                    elif erro_xy_atual <= 0.001:
                        if self.tempo_cravado is None:
                            self.tempo_cravado = agora
                        elif agora - self.tempo_cravado >= 0.5:
                            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
                            self.iniciou_blind_drop = True
                            self.get_logger().info(f"AJUSTE FINAL CONCLUÍDO (Yaw travado, Erro={erro_xy_atual*100:.2f}cm)! Iniciando Blind Drop.")
                            return
                        texto_status = "ESTABILIZANDO PARADA (YAW OK, XY <0.1cm)..."
                        cor_status = (0, 255, 0)
                    else:
                        self.tempo_cravado = None
                        texto_status = f"AGUARDANDO ALINHAMENTO (<0.1cm) | Atual: {erro_xy_atual*100:.2f}cm"
                        cor_status = (0, 255, 255)
                else:
                    self.tempo_cravado = None
                    yaw_info = "YAW OK" if self.rotacao_concluida else (f"YAW: {erro_angular_deg:.1f}°" if erro_angular_deg is not None else "SEM 3 ALÇAS")
                    texto_status = f"AJUSTE FINAL [{yaw_info}] - {tempo_restante:.1f}s"
                    cor_status = (0, 255, 255)
                    
                vel_z = 0.0 # Hover a 0.75m para permitir o ajuste fino
            else:
                self.tempo_borda_ok = None
                self.tempo_cravado = None
                
                # Descida normal da Fase 2 centralizando no offset (sem rotação)
                vel_z = 0.20
                texto_status = "FASE 2: DESCENDO NO OFFSET (Ajuste Final em 0.75m)..."
                cor_status = (0, 255, 0)
                    
            # Envia velocidade translacional e taxa de Yaw para o drone
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z, yaw_rate)
        else:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
            texto_status = "ALVO PERDIDO (Aguardando)"
            cor_status = (0, 0, 255)
            self.tempo_yaw_ok = None
            
        cv2.putText(cv_image, f"Alt Rel: {altitude_relativa:.2f}m | {texto_status}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, cor_status, 2)
                    
        if 'erro_xy_atual' in locals():
            cv2.putText(cv_image, f"Erro Offset: {erro_xy_atual*100:.2f} cm", (20, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, cor_status, 2)
                        
        if erro_angular_deg is not None:
            if tocar_borda:
                status_yaw_str = "TRAVADO" if self.rotacao_concluida else "ALINHANDO"
            else:
                status_yaw_str = "AGUARDANDO 0.75m"
            cv2.putText(cv_image, f"Erro Yaw: {erro_angular_deg:.1f}° [{status_yaw_str}]", (20, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                        
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)
