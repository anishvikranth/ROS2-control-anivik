#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit import MoveItPy
class Arm(Node):
    def __init__(self):
        super().__init__('moveit_example')
        self.moveit = MoveItPy(node_name = "moveit_py")
        self.arm = self.moveit.get_planning_component("arm")
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state()
def main(args=None):
    rclpy.init(args=args)
    node = Arm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()