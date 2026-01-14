from dataclasses import dataclass
import random

import rclpy
from rclpy.node import Node

from alc_interfaces.msg import LearnerState


@dataclass
class LearnerParams:
    knowledge: float = 0.2
    cognitive_load: float = 0.3
    error_rate: float = 0.4
    disability_profile: str = "none"


class LearnerModelNode(Node):
    """
    Publishes a synthetic LearnerState at a fixed rate.

    Best practice:
    - Keep simulation-only logic in one place (this node)
    - Make everything parameter-driven for reproducibility
    """

    def __init__(self) -> None:
        super().__init__("learner_model_node")

        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("disability_profile", "none")

        disability_profile = self.get_parameter("disability_profile").get_parameter_value().string_value
        self.state = LearnerParams(disability_profile=disability_profile)

        self.pub = self.create_publisher(LearnerState, "/learner/state", 10)

        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        period = 1.0 / max(publish_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"Started learner_model_node with disability_profile={disability_profile}")

    def _tick(self) -> None:
        # Simple synthetic dynamics (replace later with your real equations)
        noise = random.uniform(-0.02, 0.02)
        self.state.knowledge = min(1.0, max(0.0, self.state.knowledge + 0.01 + noise))

        # Disability increases baseline cognitive load (placeholder)
        base_load = 0.25 if self.state.disability_profile == "none" else 0.35
        self.state.cognitive_load = min(1.0, max(0.0, base_load + random.uniform(-0.03, 0.03)))

        # Error rate decreases as knowledge grows, increases with load
        self.state.error_rate = min(1.0, max(0.0, 0.6 - 0.5 * self.state.knowledge + 0.4 * self.state.cognitive_load))

        msg = LearnerState()
        msg.knowledge = float(self.state.knowledge)
        msg.cognitive_load = float(self.state.cognitive_load)
        msg.error_rate = float(self.state.error_rate)
        msg.disability_profile = self.state.disability_profile

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LearnerModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
