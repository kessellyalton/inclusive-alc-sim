import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from alc_interfaces.msg import LearnerState, TutorAction


class LoggerNode(Node):
    """
    Experiment logger:
    - Subscribes to key topics
    - Publishes JSONL log events to /log/events
    - Writes JSONL file to disk (reproducible experiments)

    Best practice:
    - One event per line (JSONL) -> easy parsing
    - File name includes timestamp to avoid overwrites
    """

    def __init__(self) -> None:
        super().__init__("logger_node")

        self.declare_parameter("log_dir", "logs")
        self.declare_parameter("run_name", "")

        log_dir = self.get_parameter("log_dir").get_parameter_value().string_value
        run_name = self.get_parameter("run_name").get_parameter_value().string_value.strip()

        os.makedirs(log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"run_{ts}.jsonl" if not run_name else f"{run_name}_{ts}.jsonl"
        self.path = os.path.join(log_dir, filename)

        self.pub = self.create_publisher(String, "/log/events", 10)

        self.sub_state = self.create_subscription(LearnerState, "/learner/state", self._on_state, 10)
        self.sub_action = self.create_subscription(TutorAction, "/tutor/action", self._on_action, 10)
        self.sub_feedback = self.create_subscription(String, "/tutor/feedback", self._on_feedback, 10)
        self.sub_disability = self.create_subscription(String, "/sim/disability_profile", self._on_disability, 10)

        self.current_disability = "none"

        self.get_logger().info(f"Logger writing to {self.path}")

    def _write_event(self, kind: str, data: dict) -> None:
        # Use ROS time if sim time is enabled (will come from /clock)
        stamp = self.get_clock().now().nanoseconds / 1e9

        event = {
            "t": stamp,
            "kind": kind,
            "data": data
        }

        line = json.dumps(event)

        # publish
        msg = String()
        msg.data = line
        self.pub.publish(msg)

        # persist
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(line + "\n")

    def _on_disability(self, msg: String) -> None:
        self.current_disability = msg.data.strip() or "none"
        self._write_event("disability_profile", {"profile": self.current_disability})

    def _on_state(self, s: LearnerState) -> None:
        self._write_event("learner_state", {
            "knowledge": float(s.knowledge),
            "cognitive_load": float(s.cognitive_load),
            "error_rate": float(s.error_rate),
            "disability_profile": str(s.disability_profile),
            "sim_disability_profile": self.current_disability
        })

    def _on_action(self, a: TutorAction) -> None:
        self._write_event("tutor_action", {
            "modality": a.modality,
            "pacing": float(a.pacing),
            "difficulty_level": int(a.difficulty_level)
        })

    def _on_feedback(self, msg: String) -> None:
        self._write_event("tutor_feedback", {"payload": msg.data})


def main() -> None:
    rclpy.init()
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
