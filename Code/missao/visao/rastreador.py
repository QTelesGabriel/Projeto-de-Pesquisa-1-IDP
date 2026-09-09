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
        self.kp = 0.8  # Ganho proporcional (aumentado para ir mais rápido)
        self.kd = 0.3  # Ganho derivativo (freia o movimento perto do alvo)
        
        self.erro_x_anterior = 0.0
        self.erro_y_anterior = 0.0
        self.ultimo_tempo = None
        
        self.zona_segura = 0.3  # 30% do centro da tela
        self.velocidade_descida = 0.4  # m/s
        
        # 3. Inscreve no tópico da câmera do Gazebo
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        
        self.nome_janela = "Visao VANT - Descida em Funil"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("Nó de rastreamento iniciado! Procurando armadilha...")

    def image_callback(self, msg):
        # Converte ROS para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        altura_img, largura_img, _ = cv_image.shape
        
        # Verifica a altitude atual
        altitude = self.vehicle.location.global_relative_frame.alt
        if altitude <= 3.0:
            self.get_logger().info("Altitude de 3 metros atingida! Parando o VANT.")
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0) # Zera as velocidades
            raise SystemExit # Encerra o loop do ROS para a próxima fase do projeto
            
        # Roda o YOLO
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.7)
        
        if len(resultados[0].boxes) > 0:
            # Pega as coordenadas da primeira caixa detectada
            box = resultados[0].boxes[0].xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, box)
            
            # Desenha a caixa na tela
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Calcula o centro da gaiola
            centro_gaiola_x = (x1 + x2) / 2
            centro_gaiola_y = (y1 + y2) / 2
            
            # Desenha o centro
            cv2.circle(cv_image, (int(centro_gaiola_x), int(centro_gaiola_y)), 5, (0, 0, 255), -1)
            
            # --- MATEMÁTICA DO FUNIL (PD) ---
            # Normaliza o erro de -1.0 a 1.0 (onde 0 é o centro exato)
            erro_x_norm = (centro_gaiola_x - (largura_img / 2)) / (largura_img / 2)
            erro_y_norm = (centro_gaiola_y - (altura_img / 2)) / (altura_img / 2)
            
            tempo_atual = time.time()
            if self.ultimo_tempo is None:
                dt = 0.1 # Valor inicial assumido
            else:
                dt = tempo_atual - self.ultimo_tempo
                if dt <= 0:
                    dt = 0.01
            self.ultimo_tempo = tempo_atual
            
            # Derivada (Taxa de variação do erro)
            derivada_x = (erro_x_norm - self.erro_x_anterior) / dt
            derivada_y = (erro_y_norm - self.erro_y_anterior) / dt
            
            self.erro_x_anterior = erro_x_norm
            self.erro_y_anterior = erro_y_norm
            
            # Mapeamento da câmera apontada para baixo (-90 pitch):
            # Eixo Y da imagem (cima/baixo) controla o eixo X do drone (frente/trás)
            # Eixo X da imagem (esquerda/direita) controla o eixo Y do drone (lados)
            vel_x = -(erro_y_norm * self.kp + derivada_y * self.kd)
            vel_y = (erro_x_norm * self.kp + derivada_x * self.kd)
            
            # Lógica do funil (Descida Contínua)
            erro_total = (erro_x_norm**2 + erro_y_norm**2)**0.5
            
            # Se o erro for menor que a zona segura, calcula descida proporcional
            if erro_total < self.zona_segura:
                # Alinhamento varia de 0.0 (no limite da zona) a 1.0 (perfeito no centro)
                alinhamento = (self.zona_segura - erro_total) / self.zona_segura
                vel_z = self.velocidade_descida * alinhamento
                
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
            # Se não achar a gaiola, fica parado no ar (Hover)
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0)
            cv2.putText(cv_image, "BUSCANDO ALVO...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)