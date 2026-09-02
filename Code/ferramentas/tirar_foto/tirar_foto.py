import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class CapturaManualROS(Node):
    def __init__(self):
        super().__init__('captura_manual_ros')
        
        # Cria a pasta 'fotos_manuais' no mesmo diretório do script
        self.diretorio_fotos = os.path.join(os.path.dirname(__file__), 'fotos_manuais')
        os.makedirs(self.diretorio_fotos, exist_ok=True)
        
        self.contador = 1
        self.bridge = CvBridge()
        
        # Janela de exibição
        self.nome_janela = "Camera SIYI A8 Mini - Aperte 'C' para Foto"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)
        
        # Se inscreve no tópico da câmera do drone
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        self.get_logger().info("="*50)
        self.get_logger().info(" SISTEMA DE CAPTURA MANUAL ROS 2")
        self.get_logger().info(" -> Pressione 'C' na janela do vídeo para TIRAR FOTO")
        self.get_logger().info(" -> Pressione 'Q' na janela do vídeo para SAIR")
        self.get_logger().info("="*50)

    def image_callback(self, msg):
        # Converte a mensagem do ROS 2 para uma imagem legível pelo OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Redimensiona apenas para a exibição na tela (não afeta a foto salva)
        imagem_exibicao = cv2.resize(cv_image, (960, 540))
        cv2.imshow(self.nome_janela, imagem_exibicao)
        
        # Monitora o teclado (A janela de vídeo precisa estar selecionada/em foco)
        tecla = cv2.waitKey(1) & 0xFF
        
        # Pressionou 'C' ou 'c'
        if tecla == ord('c') or tecla == ord('C'):
            agora = datetime.now().strftime("%Y%m%d_%H%M%S")
            nome_arquivo = f"captura_{self.contador:03d}_{agora}.jpg"
            caminho_completo = os.path.join(self.diretorio_fotos, nome_arquivo)
            
            # Salva a foto com qualidade máxima na resolução original (1080p)
            cv2.imwrite(caminho_completo, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
            self.get_logger().info(f"[+] Foto {self.contador:03d} salva com sucesso: {nome_arquivo}")
            
            self.contador += 1
            
        # Pressionou 'Q' ou 'q'
        elif tecla == ord('q') or tecla == ord('Q'):
            self.get_logger().info("Encerrando captura...")
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    extrator = CapturaManualROS()
    
    try:
        rclpy.spin(extrator)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        extrator.get_logger().info("Interrompido pelo terminal.")
    finally:
        cv2.destroyAllWindows()
        extrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()