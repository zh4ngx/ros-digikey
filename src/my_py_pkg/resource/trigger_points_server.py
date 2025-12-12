import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point
from my_interfaces.srv import TriggerPoints
import random


class TriggerPointsServer(Node):
    """TriggerPoints service node that generates random points"""

    def __init__(self):
        # Call the Node class constructor with the node name
        super().__init__("trigger_points_server")

        # Create a service object
        self.srv = self.create_service(
            TriggerPoints,
            "trigger_points",
            self._server_callback,
        )

    def _server_callback(self, req, resp):
        """Respond with random point"""
        # Log the request
        self.get_logger().info("Incoming request\na: %d" % (req.num_points))

        if req.num_points > 0:
            resp.success = True
        else:
            resp.success = False

        resp.points = []
        for _ in range(req.num_points):
            point = Point()
            point.x = random.uniform(-1.00, 1.00)
            point.y = random.uniform(-1.00, 1.00)
            point.z = random.uniform(-1.00, 1.00)
            resp.points.append(point)

        return resp


def main():
    rclpy.init()
    node = TriggerPointsServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
