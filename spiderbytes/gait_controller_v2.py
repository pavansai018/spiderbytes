import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

JOINTS = [
    "joint_rotation_1", "joint_limb_1", "joint_leg_1",
    "joint_rotation_2", "joint_limb_2", "joint_leg_2",
    "joint_rotation_3", "joint_limb_3", "joint_leg_3",
    "joint_rotation_4", "joint_limb_4", "joint_leg_4"
]

class KeepPublishing(Node):
    def __init__(self):
        super().__init__("keep_publishing")
        
        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )
        
        # Simple pattern that repeats
        self.pattern = [
            [0.3, 0.0, 0.0, 0.3, 0.0, 0.0, -0.3, 0.0, 0.0, -0.3, 0.0, 0.0],  # Pattern 1
            [0.4, 0.2, 0.0, 0.3, 0.0, 0.0, -0.4, 0.2, 0.0, -0.3, 0.0, 0.0],  # Pattern 2
            [0.3, 0.0, 0.0, 0.4, 0.2, 0.0, -0.3, 0.0, 0.0, -0.4, 0.2, 0.0],  # Pattern 3
            [0.2, 0.0, 0.0, 0.3, 0.0, 0.0, -0.2, 0.0, 0.0, -0.3, 0.0, 0.0],  # Pattern 4
        ]
        
        self.pattern_index = 0
        
        # Timer that runs forever
        self.timer = self.create_timer(1.0, self.publish_next)
        
        self.get_logger().info("Starting continuous publisher")
    
    def publish_next(self):
        """Publish next pattern in sequence"""
        positions = self.pattern[self.pattern_index]
        
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points = [point]
        self.pub.publish(msg)
        
        self.get_logger().info(f"Published pattern {self.pattern_index + 1}")
        
        # Move to next pattern
        self.pattern_index = (self.pattern_index + 1) % len(self.pattern)

def main():
    rclpy.init()
    node = KeepPublishing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()