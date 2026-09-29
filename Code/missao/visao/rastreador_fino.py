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
from controle_voo.gimbal import apontar_gimbal_nadir

class RastreadorFinoYOLO(Node):
    def __init__(self, vehicle):
        super().__init__('rastreador_fino_yolo')
        self.vehicle = vehicle
        self.bridge = CvBridge()
        
        # 1. Carrega o modelo YOLO da Fase 2
        # ATENÇÃO: Atualize o nome da pasta do seu novo modelo aqui!
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', '..', 'modelos_IA', 'YOLOv11n_Pose_Armadilha_TCC', 'weights', 'best.pt')
        self.get_logger().info(f"Carregando modelo YOLO FASE 2 de: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)
        
        # 2. Configurações de Controle (Ajuste Fino)
        # Ganhos bem menores que a Fase 1 para evitar oscilações brutas de perto
        self.kp = 0.2  
        self.kd = 0.05 
        self.ki = 0.005 # Ganho integral BEM mais suave para o Momento 2
        
        self.integral_x = 0.0
        self.integral_y = 0.0
        
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
        self.offset_pct_x = -0.0041
        self.offset_pct_y = 0.1542
        
        self.altitude_captura = 0.05 # Altitude RELATIVA que consideramos "Toque" na gaiola (Captura)
        
        self.erro_final_x = None
        self.erro_final_y = None
        self.iniciou_blind_drop = False
        
        # Variáveis da Máquina de Estados de Descida
        self.rotacao_concluida = False
        self.tempo_alinhamento_ok = None
        self.tempo_borda_ok = None
        
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

        # --- PASSO 3: FUNIL VISUAL (Visão Ativa - POSE) ---
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.7)
        
        centro_x, centro_y = None, None
        alcas = []
        
        # Desenha a mira (Centro da tela)
        cv2.line(cv_image, (int(self.cx) - 20, int(self.cy)), (int(self.cx) + 20, int(self.cy)), (255, 255, 255), 2)
        cv2.line(cv_image, (int(self.cx), int(self.cy) - 20), (int(self.cx), int(self.cy) + 20), (255, 255, 255), 2)
        
        img_h, img_w = cv_image.shape[:2]
        tocar_borda = False
        
        if len(resultados[0].boxes) > 0:
            # 1. Analisa a Bounding Box (A gaiola inteira)
            x1_raw, y1_raw, x2_raw, y2_raw = map(float, resultados[0].boxes[0].xyxy[0].cpu().numpy())
            
            # Aplica o Filtro EMA para matar os tremores da Bounding Box
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
            
            cv2.circle(cv_image, (int(box_cx), int(box_cy)), 4, (255, 255, 0), -1) # Centro geométrico da BB
            
            # 2. Calcula o Alvo Virtual com Offset Proporcional
            # offset_pct_x positivo = Direita, offset_pct_y positivo = Cima (Y negativo no pixel)
            alvo_x = box_cx + (box_w * self.offset_pct_x)
            alvo_y = box_cy - (box_h * self.offset_pct_y)
            
            centro_x, centro_y = alvo_x, alvo_y
            cv2.circle(cv_image, (int(alvo_x), int(alvo_y)), 8, (0, 0, 255), -1)
            cv2.putText(cv_image, "ALVO VIRTUAL", (int(alvo_x)+10, int(alvo_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            
            # 3. Transição para Ajuste Final (Momento 2)
            # Aciona quando o drone chegar a exatos 0.8 metro (80 centímetros) de ALTITUDE ABSOLUTA
            if altitude_bruta <= 0.8:
                tocar_borda = True
            
            # 4. Extrai os Keypoints apenas para achar a Rotação (Modelo Retreinado com 3 pontos)
            if resultados[0].keypoints is not None:
                kpts = resultados[0].keypoints.xy[0].cpu().numpy()
                # Coleta até 3 pontos, preservando estritamente a ordem de anotação
                pts_validos = [k for k in kpts[:3] if k[0] > 0 and k[1] > 0]
                
                filtros_pts = [self.filtro_p0, self.filtro_p1, self.filtro_p2]
                
                for i, p in enumerate(pts_validos):
                    px, py = p[0], p[1]
                    
                    if self.usar_filtro:
                        # Converte para metros para a matriz do Kalman funcionar
                        x_m_kpt = (px - self.cx) * altitude_relativa / self.fx
                        y_m_kpt = (py - self.cy) * altitude_relativa / self.fy
                        
                        f_x, f_y = filtros_pts[i].atualizar(x_m_kpt, y_m_kpt, dt)
                        
                        if f_x is not None and f_y is not None:
                            # Converte de volta para pixel
                            px = (f_x * self.fx / altitude_relativa) + self.cx
                            py = (f_y * self.fy / altitude_relativa) + self.cy
                            
                            # Desenha a bolinha Ciano mostrando onde está o ponto filtrado
                            cv2.circle(cv_image, (int(px), int(py)), 6, (255, 255, 0), -1)
                            cv2.putText(cv_image, f"F_P{i}", (int(px)+10, int(py)+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)
                    
                    alcas.append((px, py))
                    # Desenha a bolinha Magenta original (bruta)
                    cv2.circle(cv_image, (int(p[0]), int(p[1])), 4, (255, 0, 255), -1)
                    cv2.putText(cv_image, f"P{i}", (int(p[0])+10, int(p[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,255), 1)
        
        # Converte o Alvo Virtual de pixel para metros (medida bruta)
        x_m_bruto, y_m_bruto = None, None
        if centro_x is not None and centro_y is not None:
            x_m_bruto = (centro_x - self.cx) * altitude_relativa / self.fx
            y_m_bruto = (centro_y - self.cy) * altitude_relativa / self.fy
            
        # Filtro EKF (Lidando com Jitter)
        if self.usar_filtro:
            erro_x, erro_y = self.filtro.atualizar(x_m_bruto, y_m_bruto, dt)
            if erro_x is not None and erro_y is not None:
                # Projeta o alvo 3D filtrado de volta para Pixel (2D) para mostrar na tela
                pixel_x_filtrado = int((erro_x * self.fx / altitude_relativa) + self.cx)
                pixel_y_filtrado = int((erro_y * self.fy / altitude_relativa) + self.cy)
                # Bolinha Azul Clara para o Alvo Filtrado
                cv2.circle(cv_image, (pixel_x_filtrado, pixel_y_filtrado), 6, (255, 255, 0), -1)
                cv2.putText(cv_image, "FILTRADO", (pixel_x_filtrado+10, pixel_y_filtrado), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
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
            
            if tocar_borda:
                max_vel = 0.05  # Velocidade super lenta (5 cm/s) para o Momento 2
                
                # Acumula o erro no tempo apenas no Momento 2
                self.integral_x += erro_x * dt
                self.integral_y += erro_y * dt
                
                # Anti-windup (limita a força máxima que a integral pode exercer)
                self.integral_x = max(-0.5, min(0.5, self.integral_x))
                self.integral_y = max(-0.5, min(0.5, self.integral_y))
                
                # PID Completo (Proporcional + Derivativo + Integral)
                vel_x = (erro_x * self.kp + derivada_x * self.kd + self.integral_x * self.ki)
                vel_y = (erro_y * self.kp + derivada_y * self.kd + self.integral_y * self.ki)
            else:
                max_vel = 0.5
                
                # Zera a integral se não estiver no Momento 2
                self.integral_x = 0.0
                self.integral_y = 0.0
                
                # Apenas PD
                vel_x = (erro_x * self.kp + derivada_x * self.kd)
                vel_y = (erro_y * self.kp + derivada_y * self.kd)
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # PD para Rotação (Yaw Rate) baseado na geometria Isósceles
            yaw_rate = 0.0
            
            # Se as 3 alças do modelo retreinado forem vistas
            if len(alcas) == 3:
                # O usuário determinou a ordem no dataset:
                # Ponto 0 e Ponto 2 são as alças mais distantes (a base do triângulo)
                # Ponto 1 é a alça inferior (a ponta do triângulo)
                
                mid_x = (alcas[0][0] + alcas[2][0]) / 2.0
                mid_y = (alcas[0][1] + alcas[2][1]) / 2.0
                
                # Vetor do meio da base apontando para a ponta (Ponto 1)
                dx = alcas[1][0] - mid_x
                dy = alcas[1][1] - mid_y
                angulo_atual_bruto = math.atan2(dy, dx)
                
                if self.usar_filtro:
                    angulo_atual = self.filtro_angulo.atualizar(angulo_atual_bruto, dt)
                else:
                    angulo_atual = angulo_atual_bruto
                
                # Se o filtro desarmou por falta de medição
                if angulo_atual is None:
                    angulo_atual = angulo_atual_bruto
                
                # Objetivo: Esse vetor deve apontar para BAIXO (Base em cima, ponta embaixo).
                # No OpenCV, apontar para Baixo (Y positivo) é um ângulo de PI/2 radianos (90 graus).
                angulo_alvo = math.pi / 2
                
                # Cálculo do menor caminho rotacional
                erro_angular = angulo_atual - angulo_alvo
                erro_angular = (erro_angular + math.pi) % (2 * math.pi) - math.pi
                
                derivada_yaw = (erro_angular - self.erro_yaw_anterior) / dt
                self.erro_yaw_anterior = erro_angular
                
                yaw_rate = erro_angular * self.kp_yaw + derivada_yaw * self.kd_yaw
                max_yaw = 0.5
                yaw_rate = max(-max_yaw, min(max_yaw, yaw_rate))
                
                menor_erro_yaw = erro_angular
            else:
                menor_erro_yaw = self.erro_yaw_anterior
            
            # --- LÓGICA DE TRANSIÇÃO PARA BLIND DROP (BORDA DA TELA) ---
            erro_xy_atual = (erro_x**2 + erro_y**2)**0.5
            
            if tocar_borda:
                agora = time.time()
                
                # MOMENTO 2: Período Obrigatório de Ajuste Final (10s)
                # O drone ficará exatos 10 segundos travado no eixo Z ajustando X e Y antes da queda cega.
                if self.tempo_borda_ok is None:
                    self.tempo_borda_ok = agora
                
                tempo_restante = max(0.0, 10.0 - (agora - self.tempo_borda_ok))
                
                # Print no terminal a cada segundo redondo
                segundos_arredondados = int(math.ceil(tempo_restante))
                if not hasattr(self, 'ultimo_seg_impresso') or self.ultimo_seg_impresso != segundos_arredondados:
                    if segundos_arredondados > 0:
                        self.get_logger().info(f"[!] AJUSTE FINAL OBRIGATÓRIO: {segundos_arredondados}s restantes...")
                    self.ultimo_seg_impresso = segundos_arredondados
                
                if tempo_restante <= 0.0:
                    # Só desce se já passou de 10s E o erro estiver menor que um milímetro (0.1cm)
                    if erro_xy_atual <= 0.001:
                        self.iniciou_blind_drop = True
                        self.get_logger().info(f"AJUSTE FINAL CONCLUIDO E ABSURDAMENTE CRAVADO (Erro={erro_xy_atual*100:.2f}cm)! Iniciando Blind Drop.")
                        return # Próximo frame fará a queda cega
                    else:
                        texto_status = f"AGUARDANDO ALINHAMENTO (<0.1cm) | Atual: {erro_xy_atual*100:.2f}cm"
                else:
                    texto_status = f"AJUSTE FINAL OBRIGATORIO - {tempo_restante:.1f}s"
                
                # Trava a altitude (hover) para terminar o pente-fino de X/Y
                vel_z = 0.0
                
                # Feedback de cores baseado no quão perto estamos de cravar
                if erro_xy_atual <= 0.001:
                    cor_status = (0, 255, 0) # Verde = pronto para descer
                else:
                    cor_status = (0, 255, 255) # Amarelo de alerta
            else:
                self.tempo_borda_ok = None
                agora = time.time()
                
                # MOMENTO 1: Trava de Alinhamento Inicial
                # O drone SÓ desce depois que conseguir um bom alinhamento de X/Y
                if not self.rotacao_concluida:
                    if erro_xy_atual < 0.12:
                        if self.tempo_alinhamento_ok is None:
                            self.tempo_alinhamento_ok = agora
                        elif agora - self.tempo_alinhamento_ok >= 1.0:
                            # Ficou 1 segundo perfeito! Trava a rotação permanentemente
                            self.rotacao_concluida = True
                            self.get_logger().info("ROTAÇÃO TRAVADA! Iniciando descida.")
                    else:
                        # Se perder o alinhamento antes de 1 segundo, zera o contador
                        self.tempo_alinhamento_ok = None
                
                if self.rotacao_concluida:
                    vel_z = 0.2
                    texto_status = "DESCENDO E ALINHANDO..."
                    cor_status = (0, 255, 0)
                else:
                    # Trava a altitude (hover) enquanto briga com o giro inicial e o alinhamento
                    vel_z = 0.0
                    texto_status = "MOMENTO 1: ALINHANDO (Aguarde 1s)..."
                    cor_status = (0, 165, 255)
            # Rotação desativada temporariamente, focando apenas no offset (yaw_rate = 0.0)
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z, 0.0)
        else:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0, 0.0)
            texto_status = "ALVO PERDIDO (Filtro Aguardando)"
            cor_status = (0, 0, 255)
            self.tempo_alinhamento_ok = None
            
        cv2.putText(cv_image, f"Alt Rel: {altitude_relativa:.2f}m | {texto_status}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor_status, 2)
                    
        if 'erro_xy_atual' in locals():
            cv2.putText(cv_image, f"Erro Alinhamento: {erro_xy_atual*100:.2f} cm", (20, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor_status, 2)
                        
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)
