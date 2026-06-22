import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time
import os


CAMERA_TOPIC = (
    '/world/iris_runway/model/iris_with_gimbal/'
    'model/gimbal/link/pitch_link/sensor/camera/image'
)

# ==========================
# ALTERE AQUI O EXPERIMENTO
# ==========================

EXPERIMENT_NAME = 'com_sombra'
# EXPERIMENT_NAME = 'sem_sombra'

SAVE_DIR = (
    f'/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/processamento_de_imagens/'
    f'dataset/{EXPERIMENT_NAME}'
)

os.makedirs(SAVE_DIR, exist_ok=True)


class CameraNode(Node):

    def __init__(self):

        super().__init__('camera_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

        self.photo_count = 0

        self.get_logger().info(
            f'Camera iniciada\nSalvando em: {SAVE_DIR}'
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        cv2.imshow('Drone Camera', frame)

        key = cv2.waitKey(1)

        # Apertar S salva foto
        if key == ord('s'):

            self.photo_count += 1

            timestamp = int(time.time() * 1000)

            filename = (
                f'{SAVE_DIR}/'
                f'img_{self.photo_count:04d}_{timestamp}.png'
            )

            cv2.imwrite(
                filename,
                frame
            )

            self.get_logger().info(
                f'Foto salva: {filename}'
            )

        # Apertar Q fecha
        elif key == ord('q'):

            cv2.destroyAllWindows()

            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
