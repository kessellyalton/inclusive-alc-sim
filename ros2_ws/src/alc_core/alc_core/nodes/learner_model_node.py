from __future__ import annotations
from typing import Optional

import random
import rclpy
from rclpy.node import Node

from alc_interfaces.msg import LearnerState, TutorAction, TrialOutcome

from alc_core.utils.models import LearnerStateVars, ModelWeights, step_dynamics
from alc_core.utils.reward import compute_reward


class LearnerModelNode(Node):
    def __init__(self) -> None:
        super().__init__("learner_model_node")

        self.declare_parameter("seed", 7)
        self.declare_parameter("disability_profile", "none")
        self.declare_parameter("publish_hz", 2.0)  # Rate limit for pedagogical timescale

        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed)

        disability_profile = str(self.get_parameter("disability_profile").value)
        self.state = LearnerStateVars(k=0.2, c=0.3, e=0.4, disability_profile=disability_profile)

        self.weights = ModelWeights()

        self.state_pub = self.create_publisher(LearnerState, "/learner/state", 10)
        self.outcome_pub = self.create_publisher(TrialOutcome, "/sim/outcome", 10)

        # Store latest action and pending outcome (updated by subscription callback)
        self.latest_action: Optional[TutorAction] = None
        self.pending_outcome: Optional[TrialOutcome] = None
        self.state_updated = False

        self.action_sub = self.create_subscription(
            TutorAction,
            "/tutor/action",
            self._on_action,
            10,
        )

        # Timer-based publishing at bounded rate (prevents runaway feedback loop)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(publish_hz, 0.1), self._tick)

        # publish initial state once
        self._publish_state()
        self.get_logger().info(f"learner_model_node ready (seed={seed}, disability_profile={disability_profile}, publish_hz={publish_hz})")

    def _publish_state(self) -> None:
        msg = LearnerState()
        msg.knowledge = float(self.state.k)
        msg.cognitive_load = float(self.state.c)
        msg.error_rate = float(self.state.e)
        msg.disability_profile = self.state.disability_profile
        self.state_pub.publish(msg)

    def _on_action(self, a: TutorAction) -> None:
        """Store latest action and compute new state (called by subscription callback)."""
        self.latest_action = a

        # Compute state update immediately (but don't publish yet)
        k_before = self.state.k

        new_state, correct, rt = step_dynamics(
            s=self.state,
            modality=a.modality,
            pacing=float(a.pacing),
            difficulty_level=int(a.difficulty_level),
            weights=self.weights,
            rng=self.rng,
        )

        delta_k = new_state.k - k_before
        r = compute_reward(
            delta_k=delta_k,
            c=new_state.c,
            e=new_state.e,
            modality=a.modality,
            disability_profile=new_state.disability_profile,
        )

        # Update state
        self.state = new_state
        self.state_updated = True

        # Prepare outcome for publishing (but don't publish yet)
        out = TrialOutcome()
        out.correct = bool(correct)
        out.response_time = float(rt)
        out.reward = float(r)
        out.step = int(self.state.step)
        self.pending_outcome = out

    def _tick(self) -> None:
        """Timer callback: publishes state and outcome at bounded rate."""
        if self.latest_action is None:
            # No action received yet, skip this tick
            return

        if self.state_updated:
            # Publish updated state
            self._publish_state()
            self.state_updated = False

        if self.pending_outcome is not None:
            # Publish outcome for RL/metrics
            self.outcome_pub.publish(self.pending_outcome)
            self.pending_outcome = None


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
