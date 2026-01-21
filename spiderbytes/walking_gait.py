#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import variables


class CrawlGait(Node):
    def __init__(self):
        super().__init__("crawl_gait")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/joint_group_position_controller/commands",
            10
        )

        # 12 joints in the same order as controller YAML
        self.q = [0.0] * 12

        # ----- TUNING -----
        self.dt = 0.01
        self.phase_steps = 25        # smoother than 15, not too slow

        # More normal force -> more usable friction
        self.knee_stance = 0.40
        self.knee_lift_delta = 0.18

        # Hip placement and push
        self.hip_forward_mag = 0.22  # swing forward placement
        self.hip_back_limit = 0.18   # how far a stance hip is allowed to drift backward
        self.hip_push_step = 0.04    # how much stance hips push backward each step (ABS target increment)

        self.joint_groups = variables.joint_groups_spider

        self.init_stance()

    # ---------------- semantics (your rule) ----------------
    def rot_sign(self, joint_name: str) -> float:
        """
        sign = -1 => +cmd rotates joint in BACKWARD sense
        sign = +1 => +cmd rotates joint in FORWARD sense
        """
        return float(variables.dog_joints_rotation_signs[joint_name])

    def hip_cmd_forward(self, hip_joint_name: str, mag: float) -> float:
        """mag>0 means forward placement command."""
        return self.rot_sign(hip_joint_name) * mag

    def hip_cmd_backward(self, hip_joint_name: str, mag: float) -> float:
        """mag>0 means backward push command."""
        return -self.rot_sign(hip_joint_name) * mag

    # ---------------- io ----------------
    def publish(self):
        msg = Float64MultiArray()
        msg.data = self.q
        self.pub.publish(msg)

    # ---------------- smooth movers (ABS targets) ----------------
    def ramp_joint_to(self, idx: int, target: float, steps: int = None):
        if steps is None:
            steps = self.phase_steps
        start = self.q[idx]
        for k in range(steps):
            u = (k + 1) / steps
            self.q[idx] = start + (target - start) * u
            self.publish()
            time.sleep(self.dt)

    def ramp_many_to(self, targets: dict, steps: int = None):
        if steps is None:
            steps = self.phase_steps
        starts = {i: self.q[i] for i in targets.keys()}
        for k in range(steps):
            u = (k + 1) / steps
            for i, tgt in targets.items():
                self.q[i] = starts[i] + (tgt - starts[i]) * u
            self.publish()
            time.sleep(self.dt)

    # ---------------- init stance ----------------
    def init_stance(self):
        # hips neutral
        for hip_idx, hip_name in variables.hip_joints.items():
            self.q[hip_idx] = self.hip_cmd_forward(hip_name, 0.0)

        # knees crouch
        for knee_idx, knee_name in variables.knee_joints.items():
            self.q[knee_idx] = self.rot_sign(knee_name) * self.knee_stance

        self.publish()
        time.sleep(1.0)

    # ---------------- primitives ----------------
    def lift_leg(self, knee_idx: int, knee_name: str):
        target = self.rot_sign(knee_name) * (self.knee_stance + self.knee_lift_delta)
        self.ramp_joint_to(knee_idx, target)

    def lower_leg(self, knee_idx: int, knee_name: str):
        target = self.rot_sign(knee_name) * self.knee_stance
        self.ramp_joint_to(knee_idx, target)

    def move_swing_hip_forward_while_lifted(self, hip_idx: int, hip_name: str):
        """Reset/Place the swing leg forward while it is in air."""
        target = self.hip_cmd_forward(hip_name, self.hip_forward_mag)
        self.ramp_joint_to(hip_idx, target)

    def apply_stance_push_once(self, swing_hip_idx: int):
        """
        PROPULSION WITHOUT SELF-CANCEL:
        Move stance hips a little more backward and KEEP them there.
        (Do NOT return to neutral while feet are planted.)
        """
        targets = {}

        for hip_idx, hip_name in variables.hip_joints.items():
            if hip_idx == swing_hip_idx:
                continue

            # Current hip "backward magnitude" estimate (based on command sign)
            # We don't need perfect conversion; we just clamp by command direction.
            # If command is already "backward direction", keep pushing a bit until limit.
            # If it's not, start from a small backward push.
            # We'll implement this by storing a desired backward magnitude per hip in command-space.

            # Compute a new backward magnitude target (clamped)
            # We keep magnitude in [0, hip_back_limit]
            # We do NOT try to infer current mag from q; we just step toward more backward each time
            # using command-space targets.
            # Practical approach: command backward with an increasing magnitude based on a hidden accumulator.
            pass

        # We will use a per-hip accumulator instead (created lazily)
        if not hasattr(self, "stance_back_mag"):
            self.stance_back_mag = {i: 0.0 for i in variables.hip_joints.keys()}

        for hip_idx, hip_name in variables.hip_joints.items():
            if hip_idx == swing_hip_idx:
                continue
            self.stance_back_mag[hip_idx] = min(
                self.hip_back_limit,
                self.stance_back_mag[hip_idx] + self.hip_push_step
            )
            targets[hip_idx] = self.hip_cmd_backward(hip_name, self.stance_back_mag[hip_idx])

        self.ramp_many_to(targets)

    def reset_hip_when_leg_swings(self, hip_idx: int, hip_name: str):
        """
        IMPORTANT:
        When this leg is lifted (no ground contact), we can safely reset its stance accumulator,
        otherwise we'd drag the base backward.
        """
        if not hasattr(self, "stance_back_mag"):
            self.stance_back_mag = {i: 0.0 for i in variables.hip_joints.keys()}
        self.stance_back_mag[hip_idx] = 0.0

        # Bring hip near neutral before placing forward (while lifted)
        self.ramp_joint_to(hip_idx, self.hip_cmd_forward(hip_name, 0.0), steps=12)

    # ---------------- step logic ----------------
    def step_one_leg(self, hip_idx: int, knee_idx: int):
        hip_name = variables.dog_joints[hip_idx]
        knee_name = variables.dog_joints[knee_idx]

        # 1) Lift
        self.lift_leg(knee_idx, knee_name)

        # 2) Reset this hip safely (it is in air now)
        self.reset_hip_when_leg_swings(hip_idx, hip_name)

        # 3) Place swing hip forward (in air)
        self.move_swing_hip_forward_while_lifted(hip_idx, hip_name)

        # 4) While swing leg is still in air, push stance hips backward ONCE and KEEP it
        self.apply_stance_push_once(swing_hip_idx=hip_idx)

        # 5) Lower (make contact)
        self.lower_leg(knee_idx, knee_name)

    def crawl_cycle(self):
        while rclpy.ok():
            for group in self.joint_groups:
                hip, knee, leg = group
                self.step_one_leg(hip, knee)

    def walk(self):
        self.get_logger().info("Starting crawl gait...")
        self.crawl_cycle()


def main():
    rclpy.init()
    node = CrawlGait()
    node.walk()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
