#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        
        # Axis 1: Left Stick Up/Down | Axis 2 (or 3): Right Stick Left/Right
        self.declare_parameter('axis_linear', 1)       
        self.declare_parameter('axis_angular', 3)      # Changed to 2 (standard Right Stick X)
        self.declare_parameter('max_linear_speed', 1.2)  # Realistic max speed (m/s)
        self.declare_parameter('max_angular_speed', 1.5) # Realistic turn speed (rad/s)
        self.declare_parameter('deadzone', 0.1)
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Joystick Teleop Node Started.')

    def apply_deadzone(self, value, threshold):
        return value if abs(value) > threshold else 0.0

    def joy_callback(self, msg: Joy):
        axis_lin = self.get_parameter('axis_linear').value
        axis_ang = self.get_parameter('axis_angular').value
        max_lin = self.get_parameter('max_linear_speed').value
        max_ang = self.get_parameter('max_angular_speed').value
        dz = self.get_parameter('deadzone').value
        
        twist = Twist()
        
        if len(msg.axes) > max(axis_lin, axis_ang):
            raw_lin = msg.axes[axis_lin]
            raw_ang = msg.axes[axis_ang]
            
            # Apply deadzone to prevent drift/trigger resting values
            lin_val = self.apply_deadzone(raw_lin, dz)
            ang_val = self.apply_deadzone(raw_ang, dz)
            
            twist.linear.x = max_lin * lin_val
            twist.angular.z = max_ang * ang_val
            
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()