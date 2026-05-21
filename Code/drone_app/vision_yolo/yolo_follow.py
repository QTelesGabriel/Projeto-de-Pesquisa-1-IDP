import asyncio
import subprocess
import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

from mavsdk import System
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    OffboardError
)

# =========================================================
# CONFIGURACOES
# =========================================================

MODEL_PATH = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/vision_yolo/train/weights/best.pt"

CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

MAX_SPEED = 0.3

DEADZONE = 10

# =========================================================
# VISAO
# =========================================================

class VisionNode(Node):

    def __init__(self):

        super().__init__('vision_node')

        self.bridge = CvBridge()

        self.frame = None

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

        self.get_logger().info("Camera iniciada")

    def image_callback(self, msg):

        try:

            self.frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            print("FRAME RECEBIDO")

        except Exception as e:

            print(f"ERRO CAMERA: {e}")


# =========================================================
# GIMBAL
# =========================================================

def olhar_para_baixo():

    print("Virando gimbal para baixo...")

    for _ in range(5):

        comando = """
        gz topic -t /gimbal/cmd_pitch \
        -m gz.msgs.Double \
        -p 'data: 1.57'
        """

        subprocess.run(comando, shell=True)

        time.sleep(0.5)

    print("Gimbal apontado para baixo!")


# =========================================================
# LOOP ROS
# =========================================================

async def ros_loop(node):

    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.01)

        await asyncio.sleep(0.01)


# =========================================================
# CONTROLE YOLO
# =========================================================

async def seguir_alvo(drone, node, model):

    print("Iniciando controle YOLO...")

    cv2.namedWindow(
        "YOLO FOLLOW",
        cv2.WINDOW_NORMAL
    )

    while True:

        await asyncio.sleep(0.03)

        if node.frame is None:

            print("SEM FRAME")

            continue

        frame = node.frame.copy()

        height, width, _ = frame.shape

        center_x = width // 2
        center_y = height // 2

        # =================================================
        # YOLO
        # =================================================

        results = model(frame, verbose=False)

        annotated = results[0].plot()

        # =================================================
        # CENTRO DA TELA
        # =================================================

        cv2.circle(
            annotated,
            (center_x, center_y),
            8,
            (255, 0, 0),
            -1
        )

        detections = results[0].boxes

        print(f"Deteccoes: {len(detections)}")

        # =================================================
        # DETECCAO
        # =================================================

        if len(detections) > 0:

            best_box = detections[0]

            x1, y1, x2, y2 = best_box.xyxy[0]

            target_x = int((x1 + x2) / 2)
            target_y = int((y1 + y2) / 2)

            # =============================================
            # DESENHOS
            # =============================================

            cv2.circle(
                annotated,
                (target_x, target_y),
                8,
                (0, 255, 0),
                -1
            )

            cv2.line(
                annotated,
                (center_x, center_y),
                (target_x, target_y),
                (0, 255, 255),
                2
            )

            # =============================================
            # ERROS
            # =============================================

            error_x = target_x - center_x
            error_y = target_y - center_y

            print(
                f"ErroX={error_x} "
                f"ErroY={error_y}"
            )

            # =========================================
            # PID VARIABLES
            # =========================================

            previous_error_x = 0
            previous_error_y = 0

            integral_x = 0
            integral_y = 0

            previous_time = time.time()

            # =============================================
            # DELTA TIME
            # =============================================

            current_time = time.time()

            dt = current_time - previous_time

            previous_time = current_time

            if dt <= 0:
                dt = 0.001

            # =============================================
            # GANHOS PID
            # =============================================

            KP = 0.002
            KI = 0
            if error_x < DEADZONE and error_y < DEADZONE:
                KD = 0
            else:   
                KD = 0.0001

            # =============================================
            # INTEGRAL
            # =============================================

            integral_x += error_x * dt
            integral_y += error_y * dt

            # =============================================
            # DERIVATIVO
            # =============================================

            derivative_x = (
                (error_x - previous_error_x) / dt
            )

            derivative_y = (
                (error_y - previous_error_y) / dt
            )

            # =============================================
            # PID FINAL
            # =============================================

            right_speed = (
                KP * error_x
                + KI * integral_x
                + KD * derivative_x
            )

            forward_speed = -(
                KP * error_y
                + KI * integral_y
                + KD * derivative_y
            )

            # =============================================
            # SALVA ERRO ANTERIOR
            # =============================================

            previous_error_x = error_x
            previous_error_y = error_y

            # =============================================
            # LIMITES
            # =============================================

            right_speed = np.clip(
                right_speed,
                -MAX_SPEED,
                MAX_SPEED
            )

            forward_speed = np.clip(
                forward_speed,
                -MAX_SPEED,
                MAX_SPEED
            )

            print(
                f"Forward={forward_speed:.2f} "
                f"Right={right_speed:.2f}"
            )

            # =============================================
            # TEXTO
            # =============================================

            cv2.putText(
                annotated,
                f"Erro X: {error_x}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.putText(
                annotated,
                f"Erro Y: {error_y}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            # =============================================
            # CONTROLE DRONE
            # =============================================

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward_m_s=float(forward_speed),
                    right_m_s=float(right_speed),
                    down_m_s=0.0,
                    yawspeed_deg_s=0.0
                )
            )

        else:

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward_m_s=0.0,
                    right_m_s=0.0,
                    down_m_s=0.0,
                    yawspeed_deg_s=0.0
                )
            )

        # =================================================
        # JANELA OPEN CV
        # =================================================

        print("MOSTRANDO FRAME")

        cv2.imshow(
            "YOLO FOLLOW",
            annotated
        )

        key = cv2.waitKey(1)

        if key == ord('q'):

            print("Saindo...")

            break

    cv2.destroyAllWindows()


# =========================================================
# MAIN
# =========================================================

async def main():

    print("Carregando YOLO...")

    model = YOLO(MODEL_PATH)

    # =====================================================
    # GIMBAL
    # =====================================================

    olhar_para_baixo()

    # =====================================================
    # ROS
    # =====================================================

    print("Inicializando ROS...")

    rclpy.init()

    node = VisionNode()

    print("Esperando camera iniciar...")

    while node.frame is None:

        rclpy.spin_once(node, timeout_sec=0.1)

        await asyncio.sleep(0.1)

    print("Camera funcionando!")

    # =====================================================
    # DRONE
    # =====================================================

    print("Conectando ao drone...")

    drone = System()

    await drone.connect(
        system_address="udp://:14550"
    )

    print("Aguardando conexao...")

    async for state in drone.core.connection_state():

        if state.is_connected:

            print("Drone conectado!")

            break

    # =====================================================
    # TAKEOFF
    # =====================================================

    print("Configurando takeoff...")

    await drone.action.set_takeoff_altitude(10.0)

    print("Armando...")

    await drone.action.arm()

    print("Takeoff...")

    await drone.action.takeoff()

    await asyncio.sleep(30)

    # =====================================================
    # OFFBOARD
    # =====================================================

    print("Iniciando OFFBOARD...")

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0.0,
            0.0,
            0.0,
            0.0
        )
    )

    try:

        await drone.offboard.start()

    except OffboardError as e:

        print(f"Erro OFFBOARD: {e}")

        await drone.action.disarm()

        return

    # =====================================================
    # EXECUCAO
    # =====================================================

    await asyncio.gather(
        ros_loop(node),
        seguir_alvo(drone, node, model)
    )


# =========================================================
# START
# =========================================================

if __name__ == "__main__":

    asyncio.run(main())