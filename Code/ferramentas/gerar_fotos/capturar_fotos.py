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
        
        self.diretorio_fotos = os.path.join(os.path.dirname(__file__), 'fotos')
        os.makedirs(self.diretorio_fotos, exist_ok=True)
        
        # Configurações do usuário
        self.limite_fotos = 1300  # Quantas fotos você quer capturar NESTA execução? Ajuste como preferir.
        self.intervalo_salvamento = 0.5  # Segundos
        
        # Variáveis de controle ajustadas
        self.fotos_nesta_sessao = 0
        self.numero_do_arquivo = 1436
        
        self.ultimo_tempo_salvo = time.time()
        self.bridge = CvBridge()
        
        self.nome_janela = "Camera SIYI A8 Mini - Visao do Drone"
        cv2.namedWindow(self.nome_janela, cv2.WINDOW_AUTOSIZE)
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        self.get_logger().info(f"Nó iniciado! Começando da foto {self.numero_do_arquivo + 1}. Salvando em: {self.diretorio_fotos}")

    def image_callback(self, msg):
        if self.fotos_nesta_sessao >= self.limite_fotos:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        imagem_exibicao = cv2.resize(cv_image, (960, 540))
        
        cv2.imshow(self.nome_janela, imagem_exibicao)
        cv2.waitKey(1)
        
        tempo_atual = time.time()
        if (tempo_atual - self.ultimo_tempo_salvo) >= self.intervalo_salvamento:
            
            # Incrementa ambos os contadores
            self.fotos_nesta_sessao += 1
            self.numero_do_arquivo += 1
            
            # Gera nome do arquivo dando continuidade
            nome_arquivo = f"foto_{self.numero_do_arquivo:04d}.jpg"
            caminho_completo = os.path.join(self.diretorio_fotos, nome_arquivo)
            
            cv2.imwrite(caminho_completo, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
            self.ultimo_tempo_salvo = tempo_atual
            
            self.get_logger().info(f"[{self.fotos_nesta_sessao}/{self.limite_fotos}] Foto salva: {nome_arquivo}")
            
            if self.fotos_nesta_sessao >= self.limite_fotos:
                self.get_logger().info(f"Meta de {self.limite_fotos} fotos atingida! Encerrando captura...")
                raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    extrator = ExtratorDeFotos()
    
    try:
        rclpy.spin(extrator)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        extrator.get_logger().info("Captura interrompida pelo usuário.")
    finally:
        cv2.destroyAllWindows()
        extrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()