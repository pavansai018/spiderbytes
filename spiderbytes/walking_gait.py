#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import variables


class CrawlGait(Node):
    def __init__(self):
        super().__init__("crawl_gait")

        # Publisher for sending joint commands
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/joint_group_position_controller/commands",  # Same topic from YAML
            10
        )

        # Joint positions array (12 joints)
        self.q = [0.0] * 12

        # Gait parameters
        self.dt = 0.02           # Time step (50 Hz)
        self.hip_step = 0.25     # Hip step size (in radians)
        self.knee_lift = 0.35    # Knee lift size (in radians)
        self.knee_stance = 0.25  # Crouch knee stance (in radians)

        # Define the joint groups (for a spider pose)
        self.joint_groups = variables.joint_groups_spider

        # Initialize the stance position for stability
        # self.init_stance()

    def init_stance(self):
        """Crouch all legs to prepare for walking (stability)."""
        for idx, joint in variables.knee_joints.items():
            self.q[idx] = (
                variables.dog_joints_rotation_signs[joint]
                * variables.spider_pose_joint_signs[joint]
                * self.knee_stance
            )
        self.publish()
        time.sleep(1.0)

    def publish(self):
        """Publish joint positions to the controller."""
        msg = Float64MultiArray()
        msg.data = self.q
        self.pub.publish(msg)

    def lift_leg(self, knee_idx, joint_name):
        """Lift a leg by increasing the knee joint angle."""
        for _ in range(15):
            self.q[knee_idx] += (
                variables.dog_joints_rotation_signs[joint_name]
                * variables.spider_pose_joint_signs[joint_name]
                * 0.02
            )
            self.publish()
            time.sleep(self.dt)

    def lower_leg(self, knee_idx, joint_name):
        """Lower the leg by decreasing the knee joint angle."""
        for _ in range(15):
            self.q[knee_idx] -= (
                variables.dog_joints_rotation_signs[joint_name]
                * variables.spider_pose_joint_signs[joint_name]
                * 0.02
            )
            self.publish()
            time.sleep(self.dt)

    def swing_leg(self, hip_idx, joint_name):
        """Swing the leg forward by moving the hip joint."""
        for _ in range(15):
            self.q[hip_idx] += (
                variables.dog_joints_rotation_signs[joint_name]
                * variables.spider_pose_joint_signs[joint_name]
                * 0.02
            )
            self.publish()
            time.sleep(self.dt)

    def step_leg(self, group, phase="lift"):
        """Perform a full leg step: lift, swing, lower."""
        hip, knee, leg = group
        hip_joint = variables.dog_joints[hip]
        knee_joint = variables.dog_joints[knee]

        # Lifting the leg
        if phase == "lift":
            self.lift_leg(knee, knee_joint)
            return "swing"
        
        # Swing the leg forward
        if phase == "swing":
            self.swing_leg(hip, hip_joint)
            return "lower"
        
        # Lower the leg back down
        if phase == "lower":
            self.lower_leg(knee, knee_joint)
            return "lift"

    def crawl_cycle(self):
        """Coordinate the movement of legs in a crawl gait pattern."""
        while rclpy.ok():
            # Perform alternating steps between the left and right legs
            for group in self.joint_groups:
                # Control each leg in a sequential manner
                phase = "lift"
                for _ in range(3):  # Repeat the phases (lift, swing, lower)
                    phase = self.step_leg(group, phase)
                    time.sleep(0.5)  # Time to wait for the leg movement to finish

    def walk(self):
        """Start the crawling gait by calling the cycle function."""
        self.get_logger().info("Starting crawl gait...")
        self.crawl_cycle()


def main():
    rclpy.init()
    node = CrawlGait()
    node.walk()  # Start the walking loop
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
