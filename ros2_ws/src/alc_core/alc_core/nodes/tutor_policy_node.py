from __future__ import annotations

import rclpy
from rclpy.node import Node
from alc_interfaces.msg import LearnerState, TutorAction


class TutorPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("tutor_policy_node")

        self.declare_parameter("mode", "inclusive_adaptive")  # "fixed" | "adaptive" | "inclusive_adaptive"
        self.mode = str(self.get_parameter("mode").value)

        self.pub = self.create_publisher(TutorAction, "/tutor/action", 10)
        self.sub = self.create_subscription(LearnerState, "/learner/state", self._on_state, 10)

        self.get_logger().info(f"tutor_policy_node started (mode={self.mode})")

    def _on_state(self, s: LearnerState) -> None:
        a = TutorAction()

        if self.mode == "fixed":
            a.modality = "visual"
            a.pacing = 1.0
            a.difficulty_level = 2

        elif self.mode == "adaptive":
            # adapt based on knowledge/load only
            if s.cognitive_load > 0.7:
                a.pacing = 0.7
                a.difficulty_level = 1
            elif s.knowledge > 0.7:
                a.pacing = 1.2
                a.difficulty_level = 4
            else:
                a.pacing = 1.0
                a.difficulty_level = 2
            a.modality = "multimodal"

        else:
            # inclusive_adaptive: adapt modality using disability_profile
            if s.disability_profile == "dyslexia":
                a.modality = "audio"
            elif s.disability_profile == "hearing_impairment":
                a.modality = "visual"
            elif s.disability_profile == "low_vision":
                a.modality = "audio"
            else:
                a.modality = "multimodal"

            # pacing/difficulty adapt
            if s.cognitive_load > 0.7:
                a.pacing = 0.7
                a.difficulty_level = 1
            elif s.knowledge > 0.7:
                a.pacing = 1.2
                a.difficulty_level = 4
            else:
                a.pacing = 1.0
                a.difficulty_level = 2

        self.pub.publish(a)


def main() -> None:
    rclpy.init()
    node = TutorPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
