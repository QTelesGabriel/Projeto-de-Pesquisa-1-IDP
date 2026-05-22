import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

import cv2
import numpy as np


class YoloVisionNode(Node):

    def __init__(self):

        super().__init__('yolo_vision_node')

        # Conversor ROS -> OpenCV
        self.bridge = CvBridge()

        # CAMINHO DO MODELO TREINADO
        self.model = YOLO(
            '/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/vision_yolo/train/weights/best.pt'
        )

        # Tópico da câmera
        self.subscription = self.create_subscription(
            Image,
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info('YOLO Vision Node iniciado')


    def image_callback(self, msg):

        # Converte imagem ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Dimensões
        height, width, _ = frame.shape

        # Centro da tela
        center_screen_x = width // 2
        center_screen_y = height // 2

        # Roda inferência YOLO
        results = self.model(frame, verbose=False)

        # Frame com desenhos do YOLO
        annotated_frame = results[0].plot()

        # Centro da tela
        cv2.circle(
            annotated_frame,
            (center_screen_x, center_screen_y),
            6,
            (255, 0, 0),
            -1
        )

        boxes = results[0].boxes

        # Se encontrou objetos
        if len(boxes) > 0:

            # Pega maior bounding box
            largest_box = max(
                boxes,
                key=lambda box: (
                    (box.xyxy[0][2] - box.xyxy[0][0]) *
                    (box.xyxy[0][3] - box.xyxy[0][1])
                )
            )

            # Coordenadas
            x1, y1, x2, y2 = largest_box.xyxy[0]

            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            # Centro do alvo
            target_x = int((x1 + x2) / 2)
            target_y = int((y1 + y2) / 2)

            # Erro visual
            error_x = target_x - center_screen_x
            error_y = target_y - center_screen_y

            # Desenha centro do alvo
            cv2.circle(
                annotated_frame,
                (target_x, target_y),
                8,
                (0, 255, 0),
                -1
            )

            # Linha alvo -> centro
            cv2.line(
                annotated_frame,
                (center_screen_x, center_screen_y),
                (target_x, target_y),
                (0, 255, 255),
                2
            )

            # Texto
            cv2.putText(
                annotated_frame,
                f'Erro X: {error_x}',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                annotated_frame,
                f'Erro Y: {error_y}',
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            # Área da bounding box
            area = (x2 - x1) * (y2 - y1)

            cv2.putText(
                annotated_frame,
                f'Area: {area}',
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            # Terminal
            self.get_logger().info(
                f'Alvo detectado | '
                f'X={target_x} '
                f'Y={target_y} '
                f'ErroX={error_x} '
                f'ErroY={error_y}'
            )

        # Mostra janela
        cv2.imshow('YOLO Drone Vision', annotated_frame)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = YoloVisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()