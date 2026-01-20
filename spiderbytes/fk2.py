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

def leg_base(leg: int) -> int:
    return (leg - 1) * 3

class TrotGait(Node):
    def __init__(self):
        super().__init__("trot_gait")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # ==========================================================
        # SIGN MAPS FROM YOUR MEASUREMENTS (base_link frame, +X forward)
        # joint_limb_i : +q moves foot forward/backward in X
        # leg1: + -> +X  (forward)
        # leg2: + -> -X  (backward)
        # leg4: + -> -X  (backward)
        # leg3: UNKNOWN in your paste (you ran limb_1 again by mistake)
        # ==========================================================
        self.hip_fwd_sign = {
            1: +1,
            2: -1,
            3: +1,   # TEMP DEFAULT: change to +1 if needed
            4: -1,
        }

        # knee sign is not yet calibrated here; we use knee mostly for lift.
        # (You can calibrate knee like you did hip, but this will already walk better.)
        self.knee_lift_sign = {
            1: -1,
            2: -1,
            3: -1,
            4: -1,
        }

        # ========= gait params =========
        self.dt = 0.02         # 50 Hz
        self.freq = 1.2        # steps per second (trot cadence)

        # Hip drives fore-aft motion (main propulsion)
        self.hip_stance_amp = 0.35   # rad: backward sweep during stance
        self.hip_swing_amp  = 0.35   # rad: forward sweep during swing

        # Knee lift (clearance)
        self.knee_lift = 0.55        # rad

        # Keep yaw joints quiet (avoid forced turning)
        self.yaw_amp = 0.0

        # Neutral pose (start from zeros)
        self.q0 = [0.0] * len(JOINTS)

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info("Trot gait started. Diagonal pairs: (1,3) and (2,4).")

    def publish(self, q, duration_sec):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start = Duration(sec=0, nanosec=int(duration_sec * 1e9))
        msg.points = [pt]
        self.pub.publish(msg)

    def phase01(self, t, offset):
        # returns phase in [0,1)
        x = (t * self.freq + offset) % 1.0
        return x

    def tick(self):
        self.t += self.dt

        # diagonal trot offsets (half-cycle apart)
        # pair A: legs 1 and 3
        # pair B: legs 2 and 4
        phase = {
            1: self.phase01(self.t, 0.0),
            3: self.phase01(self.t, 0.0),
            2: self.phase01(self.t, 0.5),
            4: self.phase01(self.t, 0.5),
        }

        # duty factor: stance fraction
        duty = 0.60  # 60% stance, 40% swing

        q = list(self.q0)

        for leg in [1, 2, 3, 4]:
            base = leg_base(leg)
            yaw_i  = base + 0
            hip_i  = base + 1   # joint_limb
            knee_i = base + 2   # joint_leg

            ph = phase[leg]
            hip_sign = self.hip_fwd_sign[leg]

            # keep yaw quiet
            q[yaw_i] = 0.0 + self.yaw_amp * math.sin(2.0 * math.pi * ph)

            if ph < duty:
                # ---------------- STANCE ----------------
                # We want foot to go BACK relative to body during stance.
                # If +hip makes foot go +X, then stance should command -hip.
                s = ph / duty  # 0..1
                # smooth backward sweep: 0 -> 1
                back = 0.5 - 0.5 * math.cos(math.pi * s)

                q[hip_i]  = (-hip_sign) * (self.hip_stance_amp * (2.0 * back - 1.0))
                q[knee_i] = 0.0  # keep near neutral on ground
            else:
                # ---------------- SWING ----------------
                s = (ph - duty) / (1.0 - duty)  # 0..1
                # forward sweep
                fwd = 0.5 - 0.5 * math.cos(math.pi * s)
                # lift: 0 -> 1 -> 0
                lift = math.sin(math.pi * s)

                q[hip_i]  = (+hip_sign) * (self.hip_swing_amp * (2.0 * fwd - 1.0))
                q[knee_i] = self.knee_lift_sign[leg] * (self.knee_lift * lift)

        self.publish(q, duration_sec=self.dt)

def main():
    rclpy.init()
    node = TrotGait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
