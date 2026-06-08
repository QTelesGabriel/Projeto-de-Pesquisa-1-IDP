import asyncio
import subprocess
import time

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from mavsdk import System
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    OffboardError
)

from hsv_tracker import HSVTracker
from precision_landing_hsv import precision_land
from takeoff import takeoff


CAMERA_TOPIC = (
    "/world/iris_runway/model/"
    "iris_with_gimbal/model/"
    "gimbal/link/pitch_link/"
    "sensor/camera/image"
)


class VisionNode(Node):

    def __init__(self):

        super().__init__("vision_node")

        self.bridge = CvBridge()

        self.frame = None

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

    def image_callback(self, msg):

        self.frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )


def olhar_para_baixo():

    for _ in range(5):

        subprocess.run(
            """
            gz topic -t /gimbal/cmd_pitch \
            -m gz.msgs.Double \
            -p 'data: 1.57'
            """,
            shell=True
        )

        time.sleep(0.5)


async def ros_loop(node):

    while rclpy.ok():

        rclpy.spin_once(
            node,
            timeout_sec=0.01
        )

        await asyncio.sleep(0.01)


async def main():

    olhar_para_baixo()

    rclpy.init()

    node = VisionNode()

    while node.frame is None:

        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )

        await asyncio.sleep(0.1)

    tracker = HSVTracker()

    drone = System()

    await drone.connect(
        system_address="udp://:14550"
    )

    async for state in drone.core.connection_state():

        if state.is_connected:
            break

    await takeoff(drone)

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0,
            0,
            0,
            0
        )
    )

    try:

        await drone.offboard.start()

    except OffboardError:

        return

    await asyncio.gather(
        ros_loop(node),
        precision_land(
            drone,
            tracker,
            node
        )
    )


if __name__ == "__main__":

    asyncio.run(main())