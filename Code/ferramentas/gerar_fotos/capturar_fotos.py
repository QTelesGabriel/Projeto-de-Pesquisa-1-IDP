import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ExtratorDeFotos(Node):
    def __init__(self):
        super().__init__('extrator_de_fotos')
        
        # Cria a pasta 'fotos' no mesmo diretório deste script
        self.diretorio_fotos = os.path.join(os.path.dirname(__file__), 'fotos')
        os.makedirs(self.diretorio_fotos, exist_ok=True)
        
        # Configurações do usuário
        self.limite_fotos = 2000
        self.intervalo_salvamento = 0.5  # Segundos
        
        # Variáveis de controle
        self.contador_fotos = 0
        self.ultimo_tempo_salvo = time.time()
        self.bridge = CvBridge()
        
        # Inscreve-se no tópico de imagem que vem da ponte ROS-Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10 # Tamanho da fila (QoS)
        )
        
        self.get_logger().info(f"Nó iniciado! Câmera conectada. Salvando em: {self.diretorio_fotos}")

    def image_callback(self, msg):
        """Função chamada automaticamente toda vez que um frame de vídeo chega do Gazebo."""
        
        # Se já atingimos o limite, ignoramos novos frames
        if self.contador_fotos >= self.limite_fotos:
            return

        # 1. Converte a imagem do formato ROS para formato OpenCV (BGR 8-bits = 1080p colorido)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. Exibe o vídeo ao vivo na tela (você não precisa mais do rqt_image_view)
        # Redimensionamos APENAS a janela de exibição para caber na tela do seu PC, 
        # a foto salva continuará sendo 1920x1080 real.
        imagem_exibicao = cv2.resize(cv_image, (960, 540))
        cv2.imshow("Camera SIYI A8 Mini - Visão do Drone", imagem_exibicao)
        cv2.waitKey(1) # Necessário para o OpenCV atualizar a janela
        
        # 3. Lógica para salvar a cada 0.5 segundos
        tempo_atual = time.time()
        if (tempo_atual - self.ultimo_tempo_salvo) >= self.intervalo_salvamento:
            self.contador_fotos += 1
            
            # Gera nomes de arquivo organizados: foto_0001.jpg, foto_0002.jpg...
            nome_arquivo = f"foto_{self.contador_fotos:04d}.jpg"
            caminho_completo = os.path.join(self.diretorio_fotos, nome_arquivo)
            
            # Salva com a compressão JPEG desativada (Qualidade 100) para não perder dados pro YOLO
            cv2.imwrite(caminho_completo, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
            
            self.ultimo_tempo_salvo = tempo_atual
            self.get_logger().info(f"[{self.contador_fotos}/{self.limite_fotos}] Foto salva: {nome_arquivo}")
            
            # Encerra o nó se bater a meta
            if self.contador_fotos >= self.limite_fotos:
                self.get_logger().info("Meta de 2000 fotos atingida! Encerrando captura...")
                raise SystemExit # Força a saída do rclpy.spin()

def main(args=None):
    rclpy.init(args=args)
    
    extrator = ExtratorDeFotos()
    
    try:
        # Mantém o nó rodando e escutando as imagens
        rclpy.spin(extrator)
    except SystemExit:
        # Saída suave quando atingir 2000 fotos
        pass
    except KeyboardInterrupt:
        extrator.get_logger().info("Captura interrompida pelo usuário.")
    finally:
        # Fecha a janela de vídeo e destrói o nó
        cv2.destroyAllWindows()
        extrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()