#!/usr/bin/env python3
"""Virtual ESP32 + arm.py for arm_real_sim.launch.py.

Stands in for the physical arm so the REAL software stack (RoverArmHardware,
real_arm_teleop_node, MoveIt) can run against Gazebo:

  /arm_target_angles (PWM, same as arm.py receives)
        -> motor model (deadband, max speed, lag, wiring direction)
        -> joint velocities -> Gazebo (/gz/motor_velocity_controller/commands)
  Gazebo joint positions (/gz/joint_states)
        -> motor angles (differential wrist, shared gripper motor)
        -> /enc_angles (degrees), /enc_counts, /arm_target_echo, /arm_esp_uptime

Like the firmware, motors stop if no PWM arrives for 1 s, and telemetry
streams at 100 Hz regardless.

Failure drills (all std_msgs):
  ros2 topic pub --once /virtual_esp/unplug_encoders std_msgs/msg/Bool "{data: true}"   # false to replug
  ros2 topic pub --once /virtual_esp/reboot std_msgs/msg/Empty                          # counts restart from 0
  ros2 topic pub --once /virtual_esp/freeze_encoder std_msgs/msg/Int32 "{data: 0}"      # motor index, -1 = none
  ros2 param set /virtual_esp enc_dir "[-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"                 # reversed encoder
Motor order: base, shoulder, elbow, bevel1, bevel2, gripper.
"""
import math
import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32MultiArray, Float64MultiArray, Int32, Int32MultiArray, UInt32

NUM_MOTORS = 6
MOTOR_NAMES = ["base", "shoulder", "elbow", "bevel1", "bevel2", "gripper"]
JOINTS = [
    "base_link_joint", "shoulder_link_joint", "elbow_link_joint", "wrist_link_joint",
    "palm_link_joint", "right_finger_back_top_link_joint",  # gripper: one actuated joint
]
PWM_MAX = 255.0
COUNTS_PER_DEG = 1000.0  # only for /enc_counts; the real factors live in arm.c


class VirtualEsp(Node):
    def __init__(self):
        super().__init__("virtual_esp")
        self.declare_parameter("pwm_deadband", 30.0)            # PWM below which a motor doesn't move
        self.declare_parameter("max_motor_vel", [1.0] * NUM_MOTORS)  # rad/s at full PWM
        self.declare_parameter("time_constant_s", 0.08)         # motor speed lag
        self.declare_parameter("motor_dir", [1.0] * NUM_MOTORS)  # -1 = motor wired backwards
        self.declare_parameter("enc_dir", [1.0] * NUM_MOTORS)    # -1 = encoder counts backwards
        self.declare_parameter("encoder_noise_deg", 0.0)
        self.declare_parameter("watchdog_s", 1.0)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("gz_joint_states_topic", "/gz/joint_states")
        self.declare_parameter("gz_command_topic", "/gz/motor_velocity_controller/commands")

        self.pwm = [0.0] * NUM_MOTORS
        self.last_cmd_time = 0.0
        self.watchdog_stopped = True
        self.motor_vel = [0.0] * NUM_MOTORS
        self.joint_pos = None
        self.offset = [0.0] * NUM_MOTORS  # motor angle at (virtual) power-on
        self.boot_time = time.monotonic()
        self.unplugged = False
        self.frozen = -1
        self.frozen_value = 0.0
        self.noise_rng = random.Random(1)

        self.create_subscription(Float32MultiArray, "/arm_target_angles", self.on_pwm, 10)
        self.create_subscription(
            JointState, self.get_parameter("gz_joint_states_topic").value, self.on_joint_states, 10)
        self.create_subscription(Bool, "/virtual_esp/unplug_encoders", self.on_unplug, 10)
        self.create_subscription(Empty, "/virtual_esp/reboot", self.on_reboot, 10)
        self.create_subscription(Int32, "/virtual_esp/freeze_encoder", self.on_freeze, 10)

        self.gz_cmd_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("gz_command_topic").value, 10)
        self.angle_pub = self.create_publisher(Float32MultiArray, "/enc_angles", 10)
        self.count_pub = self.create_publisher(Int32MultiArray, "/enc_counts", 10)
        self.echo_pub = self.create_publisher(Float32MultiArray, "/arm_target_echo", 10)
        self.uptime_pub = self.create_publisher(UInt32, "/arm_esp_uptime", 10)

        self.add_on_set_parameters_callback(self.on_set_params)

        self.dt = 1.0 / self.get_parameter("rate_hz").value
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            "Virtual ESP32 running: PWM on /arm_target_angles -> Gazebo, Gazebo -> /enc_angles. "
            f"deadband={self.get_parameter('pwm_deadband').value:.0f} PWM, "
            f"max speed={self.get_parameter('max_motor_vel').value} rad/s.")

    # -- inputs ------------------------------------------------------------
    def on_pwm(self, msg):
        n = min(len(msg.data), NUM_MOTORS)
        self.pwm[:n] = [float(v) for v in msg.data[:n]]
        self.last_cmd_time = time.monotonic()
        if self.watchdog_stopped:
            self.watchdog_stopped = False
            self.get_logger().info("Commands arriving (LED green).")

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            if self.joint_pos is None:
                self.get_logger().info("Receiving Gazebo joint states.")
            self.joint_pos = [pos[j] for j in JOINTS]

    def on_unplug(self, msg):
        self.unplugged = msg.data
        self.get_logger().warn("DRILL: encoders UNPLUGGED (no /enc_angles)." if msg.data
                               else "DRILL: encoders replugged.")

    def on_reboot(self, _msg):
        # Encoder counts restart from 0 at the current pose and uptime restarts;
        # the PWM state is lost.
        self.offset = self.motor_angles_from_joints(self.joint_pos) if self.joint_pos else [0.0] * NUM_MOTORS
        self.boot_time = time.monotonic()
        self.pwm = [0.0] * NUM_MOTORS
        self.watchdog_stopped = True
        self.get_logger().warn("DRILL: ESP32 REBOOTED (encoder zero moved to the current pose).")

    def on_freeze(self, msg):
        self.frozen = msg.data if 0 <= msg.data < NUM_MOTORS else -1
        self.frozen_value = None
        self.get_logger().warn(
            f"DRILL: {MOTOR_NAMES[self.frozen]} encoder FROZEN." if self.frozen >= 0
            else "DRILL: no encoder frozen.")

    def on_set_params(self, params):
        # Flipping enc_dir mid-run makes that encoder count backwards FROM ITS
        # CURRENT READING (like a reversed encoder), instead of jumping to the
        # mirrored angle -- which RoverArmHardware would flag as a JUMP.
        for p in params:
            if p.name == "enc_dir" and self.joint_pos is not None:
                old = self.get_parameter("enc_dir").value
                angles = self.motor_angles_from_joints(self.joint_pos)
                for m, (o, n) in enumerate(zip(old, p.value)):
                    if o != n:
                        # keep o*(a - off) == n*(a - off_new)
                        self.offset[m] = angles[m] - (o / n) * (angles[m] - self.offset[m])
                        self.get_logger().warn(f"DRILL: {MOTOR_NAMES[m]} encoder now counts "
                                               f"{'BACKWARDS' if n < 0 else 'forwards'}.")
        return SetParametersResult(successful=True)

    # -- model ---------------------------------------------------------------
    @staticmethod
    def motor_angles_from_joints(q):
        # Same mapping as RoverArmHardware (inverse of its forward kinematics).
        base, shoulder, elbow, pitch, roll, gripper = q
        return [base, shoulder, elbow, pitch + roll, pitch - roll, gripper]

    @staticmethod
    def joint_vels_from_motors(v):
        return [v[0], v[1], v[2], (v[3] + v[4]) / 2.0, (v[3] - v[4]) / 2.0, v[5]]

    def pwm_to_speed(self, m, pwm):
        deadband = self.get_parameter("pwm_deadband").value
        max_vel = self.get_parameter("max_motor_vel").value[m]
        mag = min(abs(pwm), PWM_MAX)
        if mag <= deadband:
            return 0.0
        return math.copysign((mag - deadband) / (PWM_MAX - deadband) * max_vel, pwm)

    def tick(self):
        now = time.monotonic()
        if not self.watchdog_stopped and now - self.last_cmd_time > self.get_parameter("watchdog_s").value:
            self.watchdog_stopped = True
            self.get_logger().warn("No PWM for 1 s: watchdog stop, all motors 0 (LED red).")
        pwm = [0.0] * NUM_MOTORS if self.watchdog_stopped else self.pwm

        motor_dir = self.get_parameter("motor_dir").value
        alpha = min(1.0, self.dt / max(self.get_parameter("time_constant_s").value, 1e-3))
        for m in range(NUM_MOTORS):
            target = motor_dir[m] * self.pwm_to_speed(m, pwm[m])
            self.motor_vel[m] += (target - self.motor_vel[m]) * alpha
        self.gz_cmd_pub.publish(Float64MultiArray(data=self.joint_vels_from_motors(self.motor_vel)))

        self.echo_pub.publish(Float32MultiArray(data=[float(p) for p in pwm]))
        self.uptime_pub.publish(UInt32(data=int((now - self.boot_time) * 1000) & 0xFFFFFFFF))

        if self.joint_pos is None or self.unplugged:
            return
        enc_dir = self.get_parameter("enc_dir").value
        noise = self.get_parameter("encoder_noise_deg").value
        angles_deg = []
        for m, a in enumerate(self.motor_angles_from_joints(self.joint_pos)):
            deg = enc_dir[m] * math.degrees(a - self.offset[m])
            if noise > 0.0:
                deg += self.noise_rng.gauss(0.0, noise)
            if m == self.frozen:
                if self.frozen_value is None:
                    self.frozen_value = deg
                deg = self.frozen_value
            angles_deg.append(deg)
        self.angle_pub.publish(Float32MultiArray(data=angles_deg))
        self.count_pub.publish(Int32MultiArray(data=[int(d * COUNTS_PER_DEG) for d in angles_deg]))


def main():
    rclpy.init()
    node = VirtualEsp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
