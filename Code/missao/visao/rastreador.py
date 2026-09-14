# visao/rastreador.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from ultralytics import YOLO

# Importa a função de movimento que criamos
from controle_voo.movimento import enviar_velocidade

class RastreadorYOLO(Node):
    def __init__(self, vehicle):
        super().__init__('rastreador_yolo')
        self.vehicle = vehicle
        self.bridge = CvBridge()
        
        # 1. Carrega o modelo YOLOv11n
        # Ajuste o caminho absoluto se necessário, mas como rodaremos do main.py, o caminho relativo deve funcionar
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', 'modelos_IA', 'YOLOv11n_Armadilha', 'weights', 'best.pt')
        self.get_logger().info(f"Carregando modelo YOLO de: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)
        
        # 2. Configurações de Controle (Controlador PD)
        self.kp = 0.8  # Aumentado para uma velocidade boa (mais ágil)
        self.kd = 0.05 # Reduzido para evitar que a variação de tempo do YOLO crie picos agressivos
        
        self.erro_x_anterior = 0.0
        self.erro_y_anterior = 0.0
        self.ultimo_tempo = None
        self.descendo = False # Estado da histerese de descida
        
        self.zona_segura = 0.4  # 40% do centro da tela (Retângulo interno)
        self.limite_fuga = 0.6  # 60% do centro da tela (Retângulo externo / histerese)
        self.velocidade_descida = 0.5  # m/s (Aumentado um pouco para descer mais rápido)
        
        # 3. Configurações do Filtro EKF
        # DESLIGADO TEMPORARIAMENTE para isolar a causa da tremedeira
        self.usar_filtro = True
        if self.usar_filtro:
            from visao.filtro import FiltroAlvoEKF
            self.filtro = FiltroAlvoEKF(dt_inicial=0.1)
        
        # 3. Inscreve no tópico da câmera do Gazebo
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        
        self.nome_janela = "Visao VANT - Descida em Funil"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("Nó de rastreamento iniciado! Procurando armadilha...")

    def image_callback(self, msg):
        # --- CÁLCULO DE TEMPO (dt) e LIMITADOR DE 20 FPS ---
        tempo_atual = time.time()
        if self.ultimo_tempo is not None:
            if (tempo_atual - self.ultimo_tempo) < 0.05:
                return # Ignora o frame para manter ~20 FPS
            dt = tempo_atual - self.ultimo_tempo
        else:
            dt = 0.05
        self.ultimo_tempo = tempo_atual
        
        # Converte ROS para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        altura_img, largura_img, _ = cv_image.shape
        
        # Verifica a altitude atual
        altitude = self.vehicle.location.global_relative_frame.alt
        if altitude <= 3.0:
            self.get_logger().info("Altitude de 3 metros atingida! Parando o VANT.")
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0) # Zera as velocidades
            raise SystemExit # Encerra o loop do ROS para a próxima fase do projeto
            
        # Roda o YOLO na imagem ORIGINAL (antes de desenharmos qualquer coisa por cima)
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.7)
        
        # --- DESENHO DAS ZONAS (HUD Visão) - RETÂNGULOS ---
        centro_img = (int(largura_img / 2), int(altura_img / 2))
        
        # Como o erro vai de 0 a 1 (sendo 1 a borda), a largura_img/2 equivale ao erro 1.0.
        # Zona Segura (0.4)
        dx_30 = int(self.zona_segura * largura_img / 2)
        dy_30 = int(self.zona_segura * altura_img / 2)
        pt1_30 = (centro_img[0] - dx_30, centro_img[1] - dy_30)
        pt2_30 = (centro_img[0] + dx_30, centro_img[1] + dy_30)
        cv2.rectangle(cv_image, pt1_30, pt2_30, (100, 255, 100), 1) # Verde Claro
        
        # Limite de Fuga (0.6)
        dx_50 = int(self.limite_fuga * largura_img / 2)
        dy_50 = int(self.limite_fuga * altura_img / 2)
        pt1_50 = (centro_img[0] - dx_50, centro_img[1] - dy_50)
        pt2_50 = (centro_img[0] + dx_50, centro_img[1] + dy_50)
        cv2.rectangle(cv_image, pt1_50, pt2_50, (0, 165, 255), 1) # Laranja
        
        # Desenha uma cruz no centro exato da imagem
        cv2.drawMarker(cv_image, centro_img, (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)
        
        centro_gaiola_x = None
        centro_gaiola_y = None
        
        if len(resultados[0].boxes) > 0:
            # Pega as coordenadas da primeira caixa detectada
            box = resultados[0].boxes[0].xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, box)
            
            # Desenha a caixa na tela
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Calcula o centro bruto da gaiola
            centro_gaiola_x = (x1 + x2) / 2
            centro_gaiola_y = (y1 + y2) / 2
            
            # Desenha o centro "bruto" (Vermelho)
            cv2.circle(cv_image, (int(centro_gaiola_x), int(centro_gaiola_y)), 5, (0, 0, 255), -1)
            
        # --- FILTRO EKF ---
        # Note que chamamos o filtro MESMO QUE A YOLO FALHE (centro = None).
        if self.usar_filtro:
            centro_gaiola_x, centro_gaiola_y = self.filtro.atualizar(centro_gaiola_x, centro_gaiola_y, dt)
            if centro_gaiola_x is not None:
                # Desenha o centro "filtrado" (Azul) para compararmos na tela
                cv2.circle(cv_image, (int(centro_gaiola_x), int(centro_gaiola_y)), 5, (255, 0, 0), -1)
            
        # Se temos uma coordenada válida
        if centro_gaiola_x is not None and centro_gaiola_y is not None:
            # --- MATEMÁTICA DO FUNIL (PD) ---
            # Normaliza o erro de -1.0 a 1.0 (onde 0 é o centro exato)
            erro_x_norm = (centro_gaiola_x - (largura_img / 2)) / (largura_img / 2)
            erro_y_norm = (centro_gaiola_y - (altura_img / 2)) / (altura_img / 2)
            
            # Derivada (Taxa de variação do erro)
            derivada_x = (erro_x_norm - self.erro_x_anterior) / dt
            derivada_y = (erro_y_norm - self.erro_y_anterior) / dt
            
            self.erro_x_anterior = erro_x_norm
            self.erro_y_anterior = erro_y_norm
            
            # Fator multiplicador de altitude
            fator_altitude = max(1.0, altitude / 10.0)
            
            # Mapeamento FINAL e paramétrico. 
            # A câmera e o referencial variam muito dependendo de como o modelo foi importado no Gazebo.
            # Aqui X controla X, e Y controla Y.
            vel_x = (erro_x_norm * self.kp + derivada_x * self.kd) * fator_altitude
            vel_y = (erro_y_norm * self.kp + derivada_y * self.kd) * fator_altitude
            
            # Limite de segurança para evitar movimentos muito agressivos
            max_vel = 1.5
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # Lógica do funil com Histerese (Em formato de RETÂNGULO)
            # A distância de Chebyshev forma retângulos perfeitos
            erro_total = max(abs(erro_x_norm), abs(erro_y_norm))
            
            if self.descendo:
                # Se já estava descendo, só para de descer se o erro passar da fuga
                if erro_total > self.limite_fuga:
                    self.descendo = False
            else:
                # Se não estava descendo, só começa quando ficar dentro da zona segura
                if erro_total < self.zona_segura:
                    self.descendo = True
                    
            if self.descendo:
                # Descida contínua. Alinhamento mapeado da borda de fuga (limite_fuga) até o centro (0)
                alinhamento = (self.limite_fuga - erro_total) / self.limite_fuga
                vel_z = self.velocidade_descida * max(0.2, alinhamento) # Garante mín de 20% da vel_descida
                texto_status = f"DESCENDO (Vz: {vel_z:.2f} m/s)"
                cor = (0, 255, 0)
            else:
                vel_z = 0.0
                texto_status = "CORRIGINDO ALINHAMENTO"
                cor = (0, 165, 255)
                
            # Envia o comando para os motores
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z)
            
            # Mostra dados na tela
            cv2.putText(cv_image, f"Alt: {altitude:.1f}m | {texto_status}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor, 2)
        else:
            # Se não achar a gaiola e não tiver filtro para segurar, fica parado no ar (Hover)
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0)
            cv2.putText(cv_image, "BUSCANDO ALVO...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)