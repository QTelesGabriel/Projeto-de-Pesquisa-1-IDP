import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class GimbalController(Node):

    def __init__(self):

        super().__init__('gimbal_controller')

        # Publisher do pitch do gimbal
        self.publisher = self.create_publisher(
            Float64,
            '/gimbal/cmd_pitch',
            10
        )

        # Timer executa continuamente
        self.timer = self.create_timer(
            0.1,
            self.publish_pitch
        )

        self.get_logger().info('Gimbal Controller iniciado')


    def publish_pitch(self):

        msg = Float64()

        # +1.57 rad = olhar para baixo
        msg.data = 1.57

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = GimbalController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()