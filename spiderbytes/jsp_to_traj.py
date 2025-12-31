# spiderbytes/spiderbytes/jsp_to_traj.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JSPToTraj(Node):
    def __init__(self):
        super().__init__('jsp_to_traj')

        self.joints = [
            'joint_rotation_1', 'joint_limb_1', 'joint_leg_1',
            'joint_rotation_2', 'joint_limb_2', 'joint_leg_2',
            'joint_rotation_3', 'joint_limb_3', 'joint_leg_3',
            'joint_rotation_4', 'joint_limb_4', 'joint_leg_4',
        ]

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

    def cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}

        pt = JointTrajectoryPoint()
        pt.positions = [msg.position[idx[j]] for j in self.joints]
        pt.time_from_start.nanosec = 200_000_000  # 0.2s

        traj = JointTrajectory()
        traj.joint_names = self.joints
        traj.points = [pt]

        self.pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = JSPToTraj()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
