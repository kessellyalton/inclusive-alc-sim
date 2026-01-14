import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DisabilitySimNode(Node):
    """
    Publishes the active disability profile for the current simulated learner(s).

    Best practice:
    - Parameter-driven, so experiments are reproducible
    - Publishes periodically so late-joining nodes can receive state
    """

    def __init__(self) -> None:
        super().__init__("disability_sim_node")

        self.declare_parameter("profile", "none")
        self.declare_parameter("publish_hz", 1.0)

        self.pub = self.create_publisher(String, "/sim/disability_profile", 10)

        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        period = 1.0 / max(hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info("Started disability_sim_node")

    def _tick(self) -> None:
        msg = String()
        msg.data = self.get_parameter("profile").get_parameter_value().string_value
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DisabilitySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
