#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

def deg_to_rad(deg):
    return deg * math.pi / 180.0

class StandPosition(Node):
    def __init__(self):
        super().__init__("stand_position")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.joint_names = [
            "joint_rotation_1", "joint_limb_1", "joint_leg_1",
            "joint_rotation_2", "joint_limb_2", "joint_leg_2",
            "joint_rotation_3", "joint_limb_3", "joint_leg_3",
            "joint_rotation_4", "joint_limb_4", "joint_leg_4",
        ]

        # Start from home (90°)
        self.positions = [deg_to_rad(0)] * 12

    def send_step(self, duration):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = self.positions
        pt.time_from_start.sec = duration

        msg.points = [pt]
        print(self.positions)
        self.pub.publish(msg)
        time.sleep(duration + 0.2)

    def execute(self):
        while self.pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for subscriber...")
            time.sleep(0.5)

        # Servo 8 → joint_rotation_3 → 45°
        self.positions[6] = deg_to_rad(-45)
        self.send_step(1)
        # Servo 12 → joint_rotation_4 → 135°
        self.positions[9] = deg_to_rad(45)
        self.send_step(1)

        # Servo 10 & 14 → legs → 30°
        self.positions[8] = deg_to_rad(-60)
        self.positions[11] = deg_to_rad(-60)
        self.send_step(5)

        # Servo 9 & 13 → limbs → 135°
        self.positions[7] = deg_to_rad(-140)
        self.positions[10] = deg_to_rad(-140)
        self.send_step(2)

        # Legs back to 90°
        self.positions[8] = deg_to_rad(0)
        self.positions[11] = deg_to_rad(0)
        self.send_step(1)

        # Limbs back to 90°
        self.positions[7] = deg_to_rad(0)
        self.positions[10] = deg_to_rad(0)
        self.send_step(1)

        self.get_logger().info("Stand position complete")

def main():
    rclpy.init()
    node = StandPosition()
    node.execute()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
