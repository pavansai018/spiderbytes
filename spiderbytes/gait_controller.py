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

def leg_idx(leg: int):
    # leg 1..4 -> base index in JOINTS
    return (leg-1)*3

class CrawlGait(Node):
    def __init__(self):
        super().__init__("crawl_gait")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )
        self.rot_sign = {1:+1, 2:+1, 3:-1, 4:-1} # rotation order of legs
        # gait params (tune)
        self.dt = 0.05               # timer period
        self.step_time = 0.15         # seconds per leg swing
        self.push = 0.20             # stance push amplitude (rad)
        self.swing = 0.40            # swing forward amplitude (rad)
        self.lift = 0.60             # lift bend amplitude (rad)

        # base "standing" pose (tune to your stable pose)
        self.q0 = [0.0]*len(JOINTS)

        # crawl order to reduce yaw
        self.order = [1, 2, 3, 4]
        self.phase_t = 0.0
        self.step_i = 0

        self.timer = self.create_timer(self.dt, self.tick)

    def publish(self, q, duration_sec=0.1):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start = Duration(sec=0, nanosec=int(duration_sec*1e9))
        msg.points = [pt]
        self.pub.publish(msg)

    def tick(self):
        # progress within current swing
        self.phase_t += self.dt
        s = min(self.phase_t / self.step_time, 1.0)  # 0..1

        swing_leg = self.order[self.step_i]

        # smooth swing profile
        # lift is 0->1->0, forward is 0->1
        lift_profile = math.sin(math.pi * s)          # 0..1..0
        forward_profile = 0.5 - 0.5*math.cos(math.pi*s)  # smooth 0..1

        q = list(self.q0)

        # stance legs: small backward push
        for leg in [1,2,3,4]:
            base = leg_idx(leg)
            rot = base + 0
            limb = base + 1
            knee = base + 2

            if leg == swing_leg:
                # SWING: lift + move forward
                q[rot]  += self.rot_sign[leg] * (+self.swing * (forward_profile - 0.5) * 2.0)
                q[limb] += +self.lift * lift_profile
                q[knee] += -self.lift * lift_profile
            else:
                # STANCE: push back slightly
                q[rot]  += self.rot_sign[leg] * (-self.push * (forward_profile - 0.5) * 2.0)
                # keep limb/knee near standing

        self.publish(q, duration_sec=self.dt)

        # next leg
        if s >= 1.0:
            self.phase_t = 0.0
            self.step_i = (self.step_i + 1) % len(self.order)

def main():
    rclpy.init()
    node = CrawlGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
