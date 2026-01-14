import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from alc_interfaces.msg import TutorAction


class TutorInterfaceNode(Node):
    """
    Converts TutorAction decisions into human-readable feedback.

    In a real system:
    - This would drive TTS, visuals, gestures, haptics, etc.
    For now:
    - Publish a textual feedback string (good for logging & debugging)
    """

    def __init__(self) -> None:
        super().__init__("tutor_interface_node")

        self.sub = self.create_subscription(
            TutorAction, "/tutor/action", self._on_action, 10
        )
        self.pub = self.create_publisher(String, "/tutor/feedback", 10)

        self.get_logger().info("Started tutor_interface_node")

    def _on_action(self, action: TutorAction) -> None:
        payload = {
            "modality": action.modality,
            "pacing": round(float(action.pacing), 3),
            "difficulty": int(action.difficulty_level),
            "message": f"Teaching at difficulty {action.difficulty_level} using {action.modality} modality (pacing={action.pacing:.2f})."
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TutorInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
