import asyncio
import subprocess
import time

import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from mavsdk import System
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    OffboardError
)

from gps_navigation import gps_navigation
from takeoff import takeoff
from yolo_tracker import YoloTracker
from precision_landing import precision_land


# =========================================================
# CONFIGURACOES
# =========================================================

MODEL_PATH = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/drone_app/vision_yolo/train/weights/best.pt"

CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"


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
# MAIN
# =========================================================

async def main():

    # =====================================================
    # GIMBAL
    # =====================================================

    olhar_para_baixo()

    # =====================================================
    # ROS
    # =====================================================

    rclpy.init()

    node = VisionNode()

    print("Esperando camera iniciar...")

    while node.frame is None:

        rclpy.spin_once(node, timeout_sec=0.1)

        await asyncio.sleep(0.1)

    print("Camera funcionando!")

    # =====================================================
    # YOLO
    # =====================================================

    tracker = YoloTracker(MODEL_PATH)

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

    await takeoff(drone)

    # =====================================================
    # GPS_NAVIGATION
    # =====================================================

    print("Iniciando navegacao GPS...")

    await gps_navigation(drone)

    print("GPS concluido!")

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
    # PRECISION LANDING
    # =====================================================

    await asyncio.gather(
        ros_loop(node),
        precision_land(
            drone,
            tracker,
            node
        )
    )


# =========================================================
# START
# =========================================================

if __name__ == "__main__":

    asyncio.run(main())