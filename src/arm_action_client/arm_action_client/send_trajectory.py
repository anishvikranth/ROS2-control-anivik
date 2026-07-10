#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

#now wait for the controller to get started
    def wait_for_server(self):
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return
        self.get_logger().info('Action server available.')
    
    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "base_link_joint",
            "shoulder_link_joint",
            "elbow_link_joint",
            "wrist_link_joint",
            "palm_link_joint",
            "left_finger_link_joint",
            "right_finger_link_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.0, 0.0,0.0, 0.0, -0.5, 0.5]
        point.time_from_start.sec = 2
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

#THe parameters of send_goal_async are (goal,feedback,uuid of the goal)
# here future is a parameter that will be used to get the result of the goal(ie accepted or rejected)
        future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)  
#THe below line is a callback function that will be called when the goal is accepted or rejected
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
      # Goalhandle is a handle (reference) to a goal that has been accepted.
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.result_future = goal_handle.get_result_async()
#The below line is a callback function that will be called when the result of the goal is received(ie success or failure)
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result   #future --->response ---> result
        self.get_logger().info('Result received: {0}'.format(result))\
        
        rclpy.shutdown()  # Shutdown the ROS client library after the node is destroyed
    
    def feedback_callback(self, feedback_msg):
        #From the feedback_msg, we can get the feedback object which contains the current desired, actual and error pos/vel of the joints
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current Desired Position: {feedback.desired.positions}\n")
        self.get_logger().info(f"Current Actual Position: {feedback.actual.positions}\n")
        self.get_logger().info(f"Current Error: {feedback.error.positions}\n")


def main(args=None):
    rclpy.init(args=args)
    node = ArmActionClient()
    node.wait_for_server()
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    

if __name__ == '__main__':  
    main()  