import rclpy
from example_interfaces.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PublisherWithParams(Node):
    """Publisher node that publishes a string message."""

    def __init__(self):
        super().__init__("publisher_with_params")

        self.declare_parameter("message", "Hello")
        self.declare_parameter("timer_period", 1.0)
        self._message = self.get_parameter("message").value
        self._timer_period = self.get_parameter("timer_period").value

        self.add_post_set_parameters_callback(self._post_parameters_callback)

        self._publisher = self.create_publisher(String, "topic", 10)

        # Create timer object
        self._timer = self.create_timer(
            self._timer_period,
            self._timer_callback,
        )

    def _timer_callback(self):
        msg = String()
        msg.data = self._message
        self._publisher.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)

    def _post_parameters_callback(self, params):
        for param in params:
            if param.name == "message":
                self._message = param.value
            elif param.name == "timer_period":
                self._timer_period = param.value
                self._timer.cancel()
                self._timer = self.create_timer(
                    self._timer_period,
                    self._timer_callback,
                )
            else:
                self.get_logger().warn("Unknown parameter: %s" % param.name)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PublisherWithParams()
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
