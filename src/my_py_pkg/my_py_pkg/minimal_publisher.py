import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.msg import String


class MinimalPublisher(Node):
    """Publisher node that publishes a string message."""

    def __init__(self):
        super().__init__("minimal_publisher")
        # Create publisher object
        self._publisher = self.create_publisher(String, "topic", 10)

        # Create timer object
        self._timer = self.create_timer(0.5, self._timer_callback)

        # Counter variable to keep track of the number of messages published
        self._counter = 0

    def _timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self._counter
        self._publisher.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        self._counter += 1


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MinimalPublisher()
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
