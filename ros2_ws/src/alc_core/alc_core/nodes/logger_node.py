from __future__ import annotations

import csv
import json
import os
import subprocess
import time
from dataclasses import asdict, dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from alc_interfaces.msg import LearnerState, TutorAction, TrialOutcome


@dataclass
class Latest:
    learner: Optional[LearnerState] = None
    action: Optional[TutorAction] = None
    outcome: Optional[TrialOutcome] = None


class LoggerNode(Node):
    """
    Logs synchronized rows to CSV whenever a TrialOutcome arrives.

    Rationale:
    - Outcome is the "step boundary" (s_t, a_t -> outcome -> s_{t+1})
    - We keep the latest learner state + latest action in memory
    - On each outcome, write one row with everything we know

    This supports:
    - Chapter 5 metrics: learning gain, overload rate, reward curves, etc.
    - Reproducibility: includes run metadata and timestamps
    """

    def __init__(self) -> None:
        super().__init__("logger_node")

        # ---- parameters (experiment metadata + output) ----
        self.declare_parameter("output_dir", os.path.join(os.path.expanduser("~"), "alc_logs"))
        self.declare_parameter("run_name", "")  # if empty, auto-generate
        self.declare_parameter("condition", "inclusive_adaptive")  # fixed|adaptive|inclusive_adaptive|rl
        self.declare_parameter("disability_profile", "none")
        self.declare_parameter("seed", 42)  # random seed for reproducibility
        self.declare_parameter("flush_every_n", 1)  # 1 = flush each row

        self.latest = Latest()
        self.rows_written = 0
        self.flush_every_n = int(self.get_parameter("flush_every_n").value)

        output_dir = str(self.get_parameter("output_dir").value)
        run_name = str(self.get_parameter("run_name").value).strip()
        condition = str(self.get_parameter("condition").value)
        disability_profile = str(self.get_parameter("disability_profile").value)
        seed = int(self.get_parameter("seed").value)

        os.makedirs(output_dir, exist_ok=True)

        if not run_name:
            run_name = time.strftime("%Y%m%d_%H%M%S")

        self.csv_path = os.path.join(output_dir, f"{run_name}.csv")
        self.metadata_path = os.path.join(output_dir, f"{run_name}_metadata.json")

        # ---- write metadata JSON for reproducibility ----
        metadata = self._collect_metadata(run_name, condition, disability_profile, seed)
        with open(self.metadata_path, "w") as f:
            json.dump(metadata, f, indent=2)
        self.get_logger().info(f"Metadata written to {self.metadata_path}")

        # ---- open CSV + write header ----
        self._fh = open(self.csv_path, "w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=self._fieldnames())
        self._writer.writeheader()
        self._fh.flush()

        self.get_logger().info(f"Logging to {self.csv_path}")
        self.get_logger().info(f"condition={condition}, disability_profile={disability_profile}")

        # store metadata for each row
        self.meta = {
            "run_name": run_name,
            "condition": condition,
            "disability_profile_param": disability_profile,
        }

        # ---- subscriptions ----
        self.create_subscription(LearnerState, "/learner/state", self._on_learner, 10)
        self.create_subscription(TutorAction, "/tutor/action", self._on_action, 10)
        self.create_subscription(TrialOutcome, "/sim/outcome", self._on_outcome, 10)

    def _collect_metadata(self, run_name: str, condition: str, disability_profile: str, seed: int) -> dict:
        """
        Collect reproducibility metadata: git commit hash, ROS distro, date, seed.
        """
        # Get git commit hash
        git_hash = "unknown"
        try:
            # Try to get commit hash from the workspace root
            workspace_root = os.path.expanduser("~/dev/inclusive-alc-sim")
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=workspace_root,
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                git_hash = result.stdout.strip()
        except Exception:
            pass

        # Get ROS distro
        ros_distro = os.environ.get("ROS_DISTRO", "unknown")

        # Get current date/time
        date_str = time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime())

        return {
            "run_name": run_name,
            "condition": condition,
            "disability_profile": disability_profile,
            "seed": seed,
            "git_commit_hash": git_hash,
            "ros_distro": ros_distro,
            "date": date_str,
            "timestamp": time.time(),
        }

    def _fieldnames(self) -> list[str]:
        # one row per outcome; include both meta + signals
        return [
            # meta
            "run_name",
            "condition",
            "disability_profile_param",
            # time
            "stamp_sec",
            "stamp_nsec",
            "wall_time",
            # learner state (latest seen)
            "knowledge",
            "cognitive_load",
            "error_rate",
            "disability_profile_state",
            # action (latest seen)
            "action_modality",
            "action_pacing",
            "action_difficulty_level",
            # outcome (step boundary)
            "step",
            "correct",
            "response_time",
            "reward",
        ]

    def _on_learner(self, msg: LearnerState) -> None:
        self.latest.learner = msg

    def _on_action(self, msg: TutorAction) -> None:
        self.latest.action = msg

    def _on_outcome(self, msg: TrialOutcome) -> None:
        self.latest.outcome = msg

        # we only log when we have at least learner + action + outcome
        if self.latest.learner is None or self.latest.action is None:
            self.get_logger().warn("Outcome received but missing learner state or action; skipping row.")
            return

        now = self.get_clock().now().to_msg()

        row = {
            **self.meta,
            "stamp_sec": int(now.sec),
            "stamp_nsec": int(now.nanosec),
            "wall_time": time.time(),

            # learner
            "knowledge": float(self.latest.learner.knowledge),
            "cognitive_load": float(self.latest.learner.cognitive_load),
            "error_rate": float(self.latest.learner.error_rate),
            "disability_profile_state": str(self.latest.learner.disability_profile),

            # action
            "action_modality": str(self.latest.action.modality),
            "action_pacing": float(self.latest.action.pacing),
            "action_difficulty_level": int(self.latest.action.difficulty_level),

            # outcome
            "step": int(msg.step),
            "correct": bool(msg.correct),
            "response_time": float(msg.response_time),
            "reward": float(msg.reward),
        }

        self._writer.writerow(row)
        self.rows_written += 1

        if self.flush_every_n > 0 and (self.rows_written % self.flush_every_n == 0):
            self._fh.flush()

    def destroy_node(self) -> bool:
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass
        return super().destroy_node()


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
