#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math
import numpy as np

class SpinQuadruped(Node):
    def __init__(self):
        super().__init__('spin_quadruped')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            "joint_rotation_1", "joint_limb_1", "joint_leg_1",
            "joint_rotation_2", "joint_limb_2", "joint_leg_2",
            "joint_rotation_3", "joint_limb_3", "joint_leg_3",
            "joint_rotation_4", "joint_limb_4", "joint_leg_4",
        ]

        self.home = [0.0] * 12
        self.pose = None
        self.turn_angle = 0.1  # radians

    def send_pose(self, positions, duration=0.5):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)

        msg.points = [pt]
        self.pub.publish(msg)
        time.sleep(duration)

    def step_rotate_each_leg(self):
        lift = 0.5
        knee = -0.0
        yaw = 0.4
        duration = 0.5

        leg_map = [
            (0, 1, 2),    # leg 1
            (3, 4, 5),    # leg 2
            (6, 7, 8),    # leg 3
            (9, 10, 11),  # leg 4
        ]

        pose = self.home.copy()

        for rot, limb, knee_j in leg_map:

            # 1. Lift leg
            pose = pose.copy()
            pose[limb] = lift
            pose[knee_j] = knee
            self.send_pose(pose, duration)

            # 2. Rotate hip (while lifted)
            pose = pose.copy()
            pose[rot] += yaw
            self.send_pose(pose, duration)

            # 3. Place leg down (KEEP hip rotation)
            pose = pose.copy()
            pose[limb] = 0.0
            pose[knee_j] = 0.0
            self.send_pose(pose, duration)


    def run(self):
        # wait for controller
        while self.pub.get_subscription_count() == 0:
            time.sleep(0.2)

        # self.spin_v2(direction='left')
        for _ in range(0,7):
            self.step_rotate_each_leg()
        self.get_logger().info("Spin motion complete")

def main():
    rclpy.init()
    node = SpinQuadruped()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
