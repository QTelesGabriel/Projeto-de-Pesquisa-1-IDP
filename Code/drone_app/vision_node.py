import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2
import numpy as np

class VisionNode(Node):

    def __init__(self):

        super().__init__('vision_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info('Vision Node iniciado')

    def image_callback(self, msg):

        # Converte ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Dimensões da imagem
        height, width, _ = frame.shape

        # Centro da tela
        center_x = width // 2
        center_y = height // 2

        # Converte BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Vermelho possui duas regiões no HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Máscaras
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        # Remove ruído
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Detecta contornos
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            # Maior contorno
            largest_contour = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(largest_contour)

            # Ignora ruído pequeno
            if area > 500:

                # Calcula momentos
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:

                    target_x = int(M["m10"] / M["m00"])
                    target_y = int(M["m01"] / M["m00"])

                    # Calcula erro
                    error_x = target_x - center_x
                    error_y = target_y - center_y

                    # Desenha contorno
                    cv2.drawContours(
                        frame,
                        [largest_contour],
                        -1,
                        (0, 255, 0),
                        3
                    )

                    # Desenha centro do alvo
                    cv2.circle(
                        frame,
                        (target_x, target_y),
                        8,
                        (0, 255, 0),
                        -1
                    )

                    # Desenha centro da tela
                    cv2.circle(
                        frame,
                        (center_x, center_y),
                        8,
                        (255, 0, 0),
                        -1
                    )

                    # Linha entre alvo e centro
                    cv2.line(
                        frame,
                        (center_x, center_y),
                        (target_x, target_y),
                        (255, 255, 0),
                        2
                    )

                    # Texto do erro
                    cv2.putText(
                        frame,
                        f'Erro X: {error_x}',
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2
                    )

                    cv2.putText(
                        frame,
                        f'Erro Y: {error_y}',
                        (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2
                    )

                    # Log no terminal
                    self.get_logger().info(
                        f'Alvo detectado | Erro X: {error_x} | Erro Y: {error_y}'
                    )

        # Mostra máscara HSV
        cv2.imshow("Mascara Vermelha", mask)

        # Mostra imagem final
        cv2.imshow("Deteccao", frame)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()