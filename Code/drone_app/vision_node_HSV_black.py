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

        # ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = frame.shape

        center_x = width // 2
        center_y = height // 2

        # =========================
        # PRÉ-PROCESSAMENTO
        # =========================

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (7, 7), 0)

        # Detecta regiões escuras (círculo preto)
        _, mask = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

        # Remove ruído
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # =========================
        # CONTORNOS
        # =========================

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:

                perimeter = cv2.arcLength(largest_contour, True)

                circularity = 4 * np.pi * (area / (perimeter * perimeter + 1e-6))

                # Filtra apenas objetos circulares
                if circularity > 0.7:

                    M = cv2.moments(largest_contour)

                    if M["m00"] != 0:

                        target_x = int(M["m10"] / M["m00"])
                        target_y = int(M["m01"] / M["m00"])

                        error_x = target_x - center_x
                        error_y = target_y - center_y

                        # =========================
                        # BOUNDING BOX QUADRADA
                        # =========================

                        x, y, w, h = cv2.boundingRect(largest_contour)

                        size = max(w, h)

                        cx = x + w // 2
                        cy = y + h // 2

                        x_new = cx - size // 2
                        y_new = cy - size // 2

                        cv2.rectangle(
                            frame,
                            (x_new, y_new),
                            (x_new + size, y_new + size),
                            (0, 255, 0),
                            2
                        )

                        # Centro do alvo
                        cv2.circle(frame, (target_x, target_y), 6, (0, 255, 0), -1)

                        # Centro da tela
                        cv2.circle(frame, (center_x, center_y), 6, (255, 0, 0), -1)

                        # Linha de erro
                        cv2.line(
                            frame,
                            (center_x, center_y),
                            (target_x, target_y),
                            (255, 255, 0),
                            2
                        )

                        # Debug visual
                        cv2.putText(
                            frame,
                            f"Erro X: {error_x}",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 255),
                            2
                        )

                        cv2.putText(
                            frame,
                            f"Erro Y: {error_y}",
                            (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 255),
                            2
                        )

                        self.get_logger().info(
                            f"Círculo detectado | erro_x={error_x} erro_y={error_y} circularidade={circularity:.2f}"
                        )

        # =========================
        # DEBUG VISUAL
        # =========================

        cv2.imshow("Mascara Preto", mask)
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