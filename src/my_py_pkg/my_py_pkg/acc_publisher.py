import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from my_interfaces.msg import Accelerometer


class AccPublisher(Node):
    """Publisher node that sends dummy accelerometer data"""

    def __init__(self):
        super().__init__("acc_publisher")
        # Create publisher object
        self._publisher = self.create_publisher(Accelerometer, "my_topic", 10)

        # Create timer object
        self._timer = self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self):
        msg = Accelerometer()
        msg.x = 0.5
        msg.y = 0.1
        msg.z = -9.8
        self._publisher.publish(msg)

        self.get_logger().info(f'Publishing: "{msg}"')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = AccPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
