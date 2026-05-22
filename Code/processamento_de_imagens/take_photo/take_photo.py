import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time


CAMERA_TOPIC = (
    '/world/iris_runway/model/iris_with_gimbal/'
    'model/gimbal/link/pitch_link/sensor/camera/image'
)


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

        self.get_logger().info('Camera iniciada')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        cv2.imshow('Drone Camera', frame)

        key = cv2.waitKey(1)

        # Apertar S salva foto
        if key == ord('s'):

            filename = f'/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/processamento_de_imagens/take_photo/photos/foto_{int(time.time())}.png'

            cv2.imwrite(filename, frame)

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