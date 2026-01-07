#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class HomePosition(Node):
    def __init__(self):
        super().__init__("home_position")

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

    def publish_once(self):
        # Optional: Wait for a subscriber to connect so the message isn't lost
        while self.pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber...')
            time.sleep(0.5)

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * 12  # Simplified 12-joint list
        pt.time_from_start.sec = 1 # Recommended: tell controller how long to take
        
        msg.points = [pt]
        
        self.pub.publish(msg)
        self.get_logger().info("Home Position Sent. Shutting down...")

def main():
    rclpy.init()
    node = HomePosition()
    
    # Execute the publish logic once
    node.publish_once()
    
    # Clean up and exit immediately
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()