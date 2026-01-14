import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from alc_interfaces.msg import LearnerState, TutorAction


class PolicyNode(Node):
    """
    Baseline policy: deterministic rule-based tutor policy.

    Why start rule-based?
    - Provides a reproducible baseline for Chapter 5 comparisons
    - RL can replace this later without changing the topic contract
    """

    def __init__(self) -> None:
        super().__init__("policy_node")

        # use_sim_time is set by launch file, don't declare it here
        self.declare_parameter("base_pacing", 1.0)
        self.declare_parameter("max_difficulty", 5)

        self.current_disability = "none"
        self.latest_state = None

        self.sub_state = self.create_subscription(
            LearnerState, "/learner/state", self._on_state, 10
        )
        self.sub_disability = self.create_subscription(
            String, "/sim/disability_profile", self._on_disability, 10
        )

        self.pub_action = self.create_publisher(TutorAction, "/tutor/action", 10)

        # Publish action at a fixed rate (keeps behaviour consistent)
        self.declare_parameter("publish_hz", 2.0)
        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / max(hz, 0.1), self._publish_action)

        self.get_logger().info("Started policy_node")

    def _on_disability(self, msg: String) -> None:
        self.current_disability = msg.data.strip() or "none"

    def _on_state(self, msg: LearnerState) -> None:
        self.latest_state = msg

    def _publish_action(self) -> None:
        if self.latest_state is None:
            return

        s = self.latest_state

        # --- Difficulty: increase with knowledge, decrease with high error ---
        max_diff = int(self.get_parameter("max_difficulty").get_parameter_value().integer_value)
        knowledge_band = int(s.knowledge * max_diff) + 1
        difficulty = max(1, min(int(knowledge_band), max_diff))

        if s.error_rate > 0.45:
            difficulty = max(1, difficulty - 1)

        # --- Pacing: slow down if cognitive load or error rate high ---
        pacing = self.get_parameter("base_pacing").get_parameter_value().double_value
        if s.cognitive_load > 0.65 or s.error_rate > 0.45:
            pacing *= 0.75
        elif s.cognitive_load < 0.35 and s.error_rate < 0.2:
            pacing *= 1.15

        # --- Modality: choose based on disability profile ---
        # (Simple baseline mapping; replace with learned policy later)
        disability = self.current_disability.lower()

        if disability in ["hearing", "hearing_impairment", "deaf"]:
            modality = "visual"
        elif disability in ["dyslexia"]:
            modality = "audio"
        elif disability in ["autism", "asd"]:
            modality = "multimodal"
            pacing *= 0.9  # slightly more stable pacing
        elif disability in ["motor", "motor_difficulty"]:
            modality = "multimodal"
            pacing *= 0.85
        else:
            modality = "multimodal"

        action = TutorAction()
        action.modality = modality
        action.pacing = float(pacing)
        action.difficulty_level = int(difficulty)

        self.pub_action.publish(action)


def main() -> None:
    rclpy.init()
    
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
