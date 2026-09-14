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
        altitude = max(0.1, self.vehicle.location.global_relative_frame.alt) # Evita divisão por zero
        if altitude <= 3.0:
            self.get_logger().info("Altitude de 3 metros atingida! Parando o VANT.")
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0) # Zera as velocidades
            raise SystemExit # Encerra o loop do ROS para a próxima fase do projeto
            
        # Roda o YOLO na imagem ORIGINAL (antes de desenharmos qualquer coisa por cima)
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.7)
        
        # Parâmetros Intrínsecos da Câmera do Gazebo (Pinhole Model)
        cx = largura_img / 2.0
        cy = altura_img / 2.0
        fx = 1124.0 # Calculado a partir de H_FOV = 1.4137 rad e W = 1920
        fy = 1124.0
        
        # --- DESENHO DAS ZONAS (HUD Visão) - PROJEÇÃO 3D PARA 2D ---
        # Agora desenhamos as zonas baseadas em METROS FÍSICOS reais
        zona_segura_m = 1.0 # 1 metro de raio
        limite_fuga_m = 1.5 # 1.5 metros de raio
        
        # Converte a zona física para pixels baseado na altitude
        dx_30 = int((zona_segura_m * fx) / altitude)
        dy_30 = int((zona_segura_m * fy) / altitude)
        pt1_30 = (int(cx) - dx_30, int(cy) - dy_30)
        pt2_30 = (int(cx) + dx_30, int(cy) + dy_30)
        cv2.rectangle(cv_image, pt1_30, pt2_30, (100, 255, 100), 1) # Verde Claro
        
        dx_50 = int((limite_fuga_m * fx) / altitude)
        dy_50 = int((limite_fuga_m * fy) / altitude)
        pt1_50 = (int(cx) - dx_50, int(cy) - dy_50)
        pt2_50 = (int(cx) + dx_50, int(cy) + dy_50)
        cv2.rectangle(cv_image, pt1_50, pt2_50, (0, 165, 255), 1) # Laranja
        
        # Desenha uma cruz no centro exato da imagem
        cv2.drawMarker(cv_image, (int(cx), int(cy)), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)
        
        x_m = None
        y_m = None
        
        if len(resultados[0].boxes) > 0:
            # Pega as coordenadas da primeira caixa detectada
            box = resultados[0].boxes[0].xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, box)
            
            # Desenha a caixa na tela
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Calcula o centro bruto da gaiola em pixels
            centro_gaiola_x = (x1 + x2) / 2.0
            centro_gaiola_y = (y1 + y2) / 2.0
            cv2.circle(cv_image, (int(centro_gaiola_x), int(centro_gaiola_y)), 5, (0, 0, 255), -1)
            
            # PBVS: Conversão de Píxeis para METROS FÍSICOS (Z-Depth)
            x_m = (centro_gaiola_x - cx) * altitude / fx
            y_m = (centro_gaiola_y - cy) * altitude / fy
            
        # --- FILTRO EKF NO ESPAÇO 3D (PBVS) ---
        # Note que passamos a métrica física (metros) para o filtro
        if self.usar_filtro:
            x_m_filtrado, y_m_filtrado = self.filtro.atualizar(x_m, y_m, dt)
            if x_m_filtrado is not None:
                # Projeta o ponto 3D filtrado de volta para 2D (Pixels) para mostrar na tela
                pixel_x_filtrado = int((x_m_filtrado * fx / altitude) + cx)
                pixel_y_filtrado = int((y_m_filtrado * fy / altitude) + cy)
                cv2.circle(cv_image, (pixel_x_filtrado, pixel_y_filtrado), 5, (255, 0, 0), -1)
        else:
            x_m_filtrado, y_m_filtrado = x_m, y_m
            
        # Se temos uma coordenada válida em metros
        if x_m_filtrado is not None and y_m_filtrado is not None:
            # --- MATEMÁTICA DO CONTROLE (PBVS) ---
            # O erro agora é a própria distância em metros!
            erro_x = x_m_filtrado
            erro_y = y_m_filtrado
            
            # Derivada (Taxa de variação do erro físico, ou seja, m/s)
            derivada_x = (erro_x - self.erro_x_anterior) / dt
            derivada_y = (erro_y - self.erro_y_anterior) / dt
            
            self.erro_x_anterior = erro_x
            self.erro_y_anterior = erro_y
            
            # Controlador PD operando sobre Metros Físicos (Não precisa de fator_altitude!)
            # kp = 0.8 significa que 1 metro de erro vai gerar 0.8 m/s de velocidade
            vel_x = (erro_x * self.kp + derivada_x * self.kd)
            vel_y = (erro_y * self.kp + derivada_y * self.kd)
            
            # Limite de segurança para evitar movimentos muito agressivos
            max_vel = 1.5
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # Lógica de Histerese baseada no distanciamento FÍSICO (Chebyshev)
            erro_total_m = max(abs(erro_x), abs(erro_y))
            
            if self.descendo:
                if erro_total_m > limite_fuga_m:
                    self.descendo = False
            else:
                if erro_total_m < zona_segura_m:
                    self.descendo = True
                    
            if self.descendo:
                # Alinhamento mapeado da borda de fuga até o centro
                alinhamento = (limite_fuga_m - erro_total_m) / limite_fuga_m
                vel_z = self.velocidade_descida * max(0.2, alinhamento)
                texto_status = f"DESCENDO (Vz: {vel_z:.2f} m/s) Erro: {erro_total_m:.1f}m"
                cor = (0, 255, 0)
            else:
                vel_z = 0.0
                texto_status = f"ALINHANDO FÍSICO Erro: {erro_total_m:.1f}m"
                cor = (0, 165, 255)
                
            # Envia o comando para os motores
            enviar_velocidade(self.vehicle, vel_x, vel_y, vel_z)
            
            cv2.putText(cv_image, f"Alt: {altitude:.1f}m | {texto_status}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor, 2)
        else:
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0)
            cv2.putText(cv_image, "BUSCANDO ALVO 3D...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        cv2.imshow(self.nome_janela, cv_image)
        cv2.waitKey(1)