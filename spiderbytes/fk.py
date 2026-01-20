import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = [
    "joint_rotation_1","joint_limb_1","joint_leg_1",
    "joint_rotation_2","joint_limb_2","joint_leg_2",
    "joint_rotation_3","joint_limb_3","joint_leg_3",
    "joint_rotation_4","joint_limb_4","joint_leg_4",
]

def j(name): return JOINTS.index(name)

class TrotGait(Node):
    def __init__(self):
        super().__init__("trot_gait")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # ---- Leg layout YOU gave ----
        self.front = {1, 4}
        self.rear  = {2, 3}

        # Diagonal pairs for trot (based on your mapping)
        self.pair_A = (1, 3)  # front(1) + rear(3)
        self.pair_B = (4, 2)  # front(4) + rear(2)

        # Hip direction signs (start all +1, flip a leg if it pushes wrong)
        self.hip_dir = {1:-1, 2:-1, 3:-1, 4:-1}

        self.dt = 0.03
        self.T  = 1.0

        self.swing_amp  = 0.35
        self.stance_amp = 0.25
        self.lift_amp   = 0.45

        # IMPORTANT: if feet are not gripping, increase knee_base so stance legs "press down"
        self.knee_base  = 0.45  # start 0.45; try 0.55 if slipping

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.tick)

    def publish(self, q):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        msg.points = [pt]
        self.pub.publish(msg)

    def tick(self):
        self.t += self.dt
        phase = (self.t % self.T) / self.T  # 0..1

        swing_A = phase < 0.5
        s = (phase*2) % 1.0

        lift = math.sin(math.pi * s)        # 0..1..0
        fwd  = math.sin(2*math.pi * s)      # -1..+1 (forward/back)

        q = [0.0]*len(JOINTS)

        # lock yaw joints
        for i in (1,2,3,4):
            q[j(f"joint_rotation_{i}")] = 0.0

        def set_leg(i, swing):
            if swing:
                hip  = self.hip_dir[i] * ( self.swing_amp * fwd)
                knee = self.knee_base + self.lift_amp * lift
            else:
                hip  = self.hip_dir[i] * (-self.stance_amp * fwd)  # stance PUSH
                knee = self.knee_base

            q[j(f"joint_limb_{i}")] = hip
            q[j(f"joint_leg_{i}")]  = knee

        # Apply swing/stance by diagonal pairs
        swing_pair  = self.pair_A if swing_A else self.pair_B
        stance_pair = self.pair_B if swing_A else self.pair_A

        for i in swing_pair:
            set_leg(i, True)
        for i in stance_pair:
            set_leg(i, False)

        self.publish(q)

def main():
    rclpy.init()
    rclpy.spin(TrotGait())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
