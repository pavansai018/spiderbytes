#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import variables

class HomePosition(Node):
    def __init__(self):
        super().__init__("home_position")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.joint_names = variables.dog_joints
        self.rotation_sign = variables.dog_joints_rotation_signs
        self.target_position = [
                variables.spider_home_angle_from_dog_pose['hip'],variables.spider_home_angle_from_dog_pose['knee'], variables.spider_home_angle_from_dog_pose['leg'],
                variables.spider_home_angle_from_dog_pose['hip'],variables.spider_home_angle_from_dog_pose['knee'], variables.spider_home_angle_from_dog_pose['leg'],
                variables.spider_home_angle_from_dog_pose['hip'],variables.spider_home_angle_from_dog_pose['knee'], variables.spider_home_angle_from_dog_pose['leg'],
                variables.spider_home_angle_from_dog_pose['hip'],variables.spider_home_angle_from_dog_pose['knee'], variables.spider_home_angle_from_dog_pose['leg'],
            ]
        self.current_position = [0.0] * 12
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
    def spider_pose(self):
        for group in variables.joint_groups_spider:
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            pt = JointTrajectoryPoint()
            pt.positions = self.current_position
            for idx in group:
                joint = self.joint_names[idx]
                pt.positions[idx] = (
                    self.rotation_sign[joint]
                    * variables.spider_pose_joint_signs[joint]
                    * self.target_position[idx]
                )
            pt.time_from_start.sec = 1
            msg.points = [pt]
            self.pub.publish(msg)
            # print(f'Group: {group}, PrePos: {pt.positions}')
            time.sleep(1.1)
            pt.positions[group[1]] = 0
            msg.points = [pt]
            self.pub.publish(msg)
            time.sleep(1.1)
            self.current_position = pt.positions
            # print(f'Group: {group}, PostPos: {pt.positions}')

        for key, value in variables.knee_joints.items():
            self.current_position[key] = self.rotation_sign[value] * variables.spider_pose_joint_signs[value] * variables.spider_crouch_position['knee']
        pt.positions = self.current_position
        pt.time_from_start.sec = 1
        msg.points = [pt]
        self.pub.publish(msg)

    def home_position(self):
        for group in variables.joint_groups_dog:
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            pt = JointTrajectoryPoint()
            pt.positions = self.current_position
            for idx in group:
                joint = self.joint_names[idx]
                pt.positions[idx] = (
                    self.rotation_sign[joint]
                    * variables.spider_pose_joint_signs[joint]
                    * 0
                )
            pt.time_from_start.sec = 1
            msg.points = [pt]
            self.pub.publish(msg)
            time.sleep(1.1)
            pt.positions[group[1]] = 0
            msg.points = [pt]
            self.pub.publish(msg)
            time.sleep(1.1)
            self.current_position = pt.positions
    def publish_once(self):
        # Optional: Wait for a subscriber to connect so the message isn't lost
        while self.pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber...')
            time.sleep(0.5)
        self.spider_pose()
        time.sleep(5)
        self.home_position()
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