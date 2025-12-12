import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):
    """Minimal client node for adding two integers."""

    def __init__(self):
        super().__init__("minimal_client")
        self._client = self.create_client(AddTwoInts, "add_ints")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self._request = AddTwoInts.Request()

        self.timer = self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self):
        """Send a random request to the server."""
        # Fill out request
        req = AddTwoInts.Request()
        req.a = random.randint(0, 10)
        req.b = random.randint(0, 10)

        # Call service asynchronously and set callback
        self.future = self._client.call_async(req)
        self.future.add_done_callback(self._response_callback)

    def _response_callback(self, future):
        """Log the response from the server."""
        try:
            response = future.result()
            self.get_logger().info(f"Result: {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MinimalClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
