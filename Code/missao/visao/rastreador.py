# visao/rastreador.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
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
        caminho_modelo = os.path.join(os.path.dirname(__file__), '..', '..', 'modelos_IA', 'YOLOv11n_Armadilha_TCC', 'weights', 'best.pt')
        self.get_logger().info(f"Carregando modelo YOLO de: {caminho_modelo}")
        self.yolo = YOLO(caminho_modelo)
        
        # 2. Configurações de Controle (Controlador PD em Cascata)
        # Kp com valor 0.4 (evita chegar muito rápido)
        # Kd com valor de 0.01 para agir como um compensador de latência visual extremamente fino.
        self.kp = 0.4  
        self.kd = 0.01 
        
        self.erro_x_anterior = 0.0
        self.erro_y_anterior = 0.0
        self.ultimo_tempo = None
        self.descendo = False # Estado da histerese de descida
        
        self.zona_segura = 0.4  # 40% do centro da tela (Retângulo interno)
        self.limite_fuga = 0.6  # 60% do centro da tela (Retângulo externo / histerese)
        self.velocidade_descida = 1.5  # m/s
        
        # 3. Configurações do Filtro EKF
        self.usar_filtro = True
        if self.usar_filtro:
            from visao.filtro import FiltroAlvoEKF
            self.filtro = FiltroAlvoEKF(dt_inicial=0.1)
            
        # Parâmetros intrínsecos dinâmicos da câmera 
        # Esses valores são iniciados com valores padrão da SIYI A8 mini, mas atualizados pelo ROS2
        self.fx = 1124.0
        self.fy = 1124.0
        self.cx = 960.0
        self.cy = 540.0
        
        # 4. Inscreve no tópico da câmera do Gazebo
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        
        # 5. Inscreve no tópico de CameraInfo para pegar os parâmetros em tempo real (Ponto 2)
        self.info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )
        
        self.nome_janela = "Visao VANT - Descida em Funil"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("Nó de rastreamento iniciado! Procurando armadilha...")

    def camera_info_callback(self, msg):
        # A matriz K é 3x3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

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
        if altitude <= 2.0:
            self.get_logger().info("Altitude de 2 metros atingida! Parando o VANT para a Fase 2.")
            enviar_velocidade(self.vehicle, 0.0, 0.0, 0.0) # Zera as velocidades
            raise SystemExit # Encerra o loop do ROS para a próxima fase do projeto
            
        # Roda o YOLO na imagem ORIGINAL (antes de desenharmos qualquer coisa por cima)
        resultados = self.yolo.predict(cv_image, verbose=False, conf=0.7)
        
        # Parâmetros Intrínsecos da Câmera do Gazebo (Pinhole Model) obtidos via Tópico ROS2
        cx = self.cx
        cy = self.cy
        fx = self.fx
        fy = self.fy
        
        # --- DESENHO DAS ZONAS (HUD Visão) - IBVS ---
        # Agora desenhamos as zonas baseadas na PROPORÇÃO DA TELA (Pixels), garantindo que o alvo nunca saia do FOV
        dx_segura = int(self.zona_segura * largura_img / 2)
        dy_segura = int(self.zona_segura * altura_img / 2)
        pt1_segura = (int(cx) - dx_segura, int(cy) - dy_segura)
        pt2_segura = (int(cx) + dx_segura, int(cy) + dy_segura)
        cv2.rectangle(cv_image, pt1_segura, pt2_segura, (100, 255, 100), 1) # Verde Claro
        
        dx_fuga = int(self.limite_fuga * largura_img / 2)
        dy_fuga = int(self.limite_fuga * altura_img / 2)
        pt1_fuga = (int(cx) - dx_fuga, int(cy) - dy_fuga)
        pt2_fuga = (int(cx) + dx_fuga, int(cy) + dy_fuga)
        cv2.rectangle(cv_image, pt1_fuga, pt2_fuga, (0, 165, 255), 1) # Laranja
        
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
            
            # Câmera apontando pela asa ESQUERDA (Gimbal -90):
            # Erro X da imagem (Esquerda/Direita) controla o eixo X do Drone (Frente/Trás)
            vel_x = (erro_x * self.kp + derivada_x * self.kd)
            
            # Erro Y da imagem (Cima/Baixo) controla o eixo Y do Drone (Lados)
            vel_y = (erro_y * self.kp + derivada_y * self.kd)
            
            # Limite de segurança para evitar movimentos muito agressivos
            max_vel = 1.5
            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            # Lógica de Histerese IBVS (Proporção da Tela)
            # Como nosso erro de controle (erro_x) está em metros, nós o convertemos de volta
            # para proporção da imagem (Pixels) apenas para a decisão de descida.
            # Isso garante que a descida só ocorra se o alvo estiver opticamente centralizado.
            erro_pixel_norm_x = (erro_x * fx / altitude) / (largura_img / 2.0)
            erro_pixel_norm_y = (erro_y * fy / altitude) / (altura_img / 2.0)
            
            erro_norm_fuga = max(abs(erro_pixel_norm_x) / self.limite_fuga, abs(erro_pixel_norm_y) / self.limite_fuga)
            erro_norm_segura = max(abs(erro_pixel_norm_x) / self.zona_segura, abs(erro_pixel_norm_y) / self.zona_segura)
            
            if self.descendo:
                if erro_norm_fuga > 1.0: # Saiu do limite de fuga ótico
                    self.descendo = False
            else:
                if erro_norm_segura < 1.0: # Entrou totalmente na zona segura ótica
                    self.descendo = True
                    
            if self.descendo:
                # Alinhamento mapeado da borda de fuga (1.0) até o centro (0.0)
                alinhamento = max(0.0, 1.0 - erro_norm_fuga)
                vel_z_base = self.velocidade_descida * max(0.2, alinhamento)
                
                # Fator de frenagem estendido (diminui a velocidade vertical entre 6m e 2m para dissipar inércia de altas velocidades)
                if altitude <= 6.0:
                    # Mapeia linearmente: 6.0m -> fator 1.0 (100%), 2.0m -> fator 0.0
                    fator_frenagem = (altitude - 2.0) / (6.0 - 2.0)
                    # Limitamos o fator em 0.1 (10%) para garantir que ele rasteje bem suave no metro final
                    fator_frenagem = max(0.1, min(1.0, fator_frenagem)) 
                else:
                    fator_frenagem = 1.0
                    
                vel_z = vel_z_base * fator_frenagem
                
                texto_status = f"DESCENDO (Vz: {vel_z:.2f} m/s) Alinhado: {(alinhamento*100):.0f}%"
                cor = (0, 255, 0)
            else:
                vel_z = 0.0
                texto_status = f"ALINHANDO FÍSICO ErroNorm: {erro_norm_segura:.1f}"
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