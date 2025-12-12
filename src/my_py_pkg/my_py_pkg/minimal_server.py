import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MinimalService(Node):
    """Minimal service node that adds two integers."""

    def __init__(self):
        # Call the Node class constructor with the node name
        super().__init__("minimal_server")

        # Create a service object
        self.srv = self.create_service(
            AddTwoInts,
            "add_ints",
            self._server_callback,
        )

    def _server_callback(self, req, resp):
        """Respond to a request with sum of two integers"""
        resp.sum = req.a + req.b
        self.get_logger().info("Incoming request\na: %d b: %d" % (req.a, req.b))
        return resp


def main():
    rclpy.init()
    node = MinimalService()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
