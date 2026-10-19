#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState


class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')

        # -------------------------------------------------------------
        # JOYSTICK MAPPINGS (Change these member variables as needed)
        # -------------------------------------------------------------
        self.axis_base_yaw = 0          # Left Stick Left/Right
        self.axis_shoulder_pitch = 1    # Left Stick Up/Down
        self.axis_elbow_pitch = 4       # Right Stick Up/Down
        self.axis_wrist_pitch = 3       # Right Stick Left/Right
        self.axis_palm_roll = 2         # Left Trigger / Twist Axis
        
        self.btn_gripper_open = 0       # Button A / Cross
        self.btn_gripper_close = 1      # Button B / Circle

        # Control speeds (radians increment per timer loop)
        self.joint_speed = 0.03
        self.gripper_speed = 0.02

        # Joint position state variables
        self.pos_base = 0.0
        self.pos_shoulder = -0.5
        self.pos_elbow = 0.0
        self.pos_wrist = 0.0
        self.pos_palm = 0.0
        self.pos_gripper = 0.0

        # URDF Joint limits extracted from rover_arm_test_version.xacro
        self.LIMIT_BASE = (-1.570795, 1.570795)
        self.LIMIT_SHOULDER = (-1.570795, 0.0)
        self.LIMIT_ELBOW = (-3.141593, 3.141593)
        self.LIMIT_WRIST = (-1.570795, 1.570795)
        self.LIMIT_GRIPPER = (-0.785398, 0.523598)

        # Store latest joystick state
        self.latest_joy = None

        # Subscriber and Publisher
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy_arm',
            self.joy_callback,
            10
        )
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Control loop running at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

    def joy_callback(self, msg: Joy):
        self.latest_joy = msg

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def control_loop(self):
        if self.latest_joy is None:
            return

        axes = self.latest_joy.axes
        buttons = self.latest_joy.buttons

        # Update base joint
        if abs(axes[self.axis_base_yaw]) > 0.1:
            self.pos_base += axes[self.axis_base_yaw] * self.joint_speed
            self.pos_base = self.clamp(self.pos_base, *self.LIMIT_BASE)

        # Update shoulder joint
        if abs(axes[self.axis_shoulder_pitch]) > 0.1:
            self.pos_shoulder += axes[self.axis_shoulder_pitch] * self.joint_speed
            self.pos_shoulder = self.clamp(self.pos_shoulder, *self.LIMIT_SHOULDER)

        # Update elbow joint
        if abs(axes[self.axis_elbow_pitch]) > 0.1:
            self.pos_elbow += axes[self.axis_elbow_pitch] * self.joint_speed
            self.pos_elbow = self.clamp(self.pos_elbow, *self.LIMIT_ELBOW)

        # Update wrist joint
        if abs(axes[self.axis_wrist_pitch]) > 0.1:
            self.pos_wrist += axes[self.axis_wrist_pitch] * self.joint_speed
            self.pos_wrist = self.clamp(self.pos_wrist, *self.LIMIT_WRIST)

        # Update continuous palm roll (no upper/lower limits)
        if abs(axes[self.axis_palm_roll]) > 0.1:
            self.pos_palm += axes[self.axis_palm_roll] * self.joint_speed

        # Update gripper (driven by right_finger_back_top_link_joint)
        if buttons[self.btn_gripper_open]:
            self.pos_gripper += self.gripper_speed
        elif buttons[self.btn_gripper_close]:
            self.pos_gripper -= self.gripper_speed
        self.pos_gripper = self.clamp(self.pos_gripper, *self.LIMIT_GRIPPER)

        # Publish target joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'base_link_joint',
            'shoulder_link_joint',
            'elbow_link_joint',
            'wrist_link_joint',
            'palm_link_joint',
            'right_finger_back_top_link_joint'
        ]
        msg.position = [
            self.pos_base,
            self.pos_shoulder,
            self.pos_elbow,
            self.pos_wrist,
            self.pos_palm,
            self.pos_gripper
        ]

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()