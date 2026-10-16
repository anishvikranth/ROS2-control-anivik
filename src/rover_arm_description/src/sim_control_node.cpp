#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>
#include <string>

class SimControlNode : public rclcpp::Node {
    public:
        SimControlNode() : rclcpp::Node("sim_control_node") {
            joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&SimControlNode::joy_callback, this, std::placeholders::_1));
            trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
            joint_names = {
                "base_link_joint",
                "shoulder_link_joint",
                "elbow_link_joint",
                "wrist_link_joint",
                "palm_link_joint",
                "left_finger_link_joint",
                "right_finger_link_joint"
            };
            joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        } 

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;
        std::vector<std::string> joint_names;
        std::vector<double> joint_positions;
        
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};      

void SimControlNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    double k = 0.03;
    joint_positions[0] += k * msg->axes[0];     // Base
    joint_positions[1] += k * msg->axes[1];     // Shoulder
    joint_positions[2] += k * msg->axes[4];     // Elbow
    joint_positions[3] += k * msg->axes[7];     // Pitch
    joint_positions[4] += k * msg->axes[6];     // Roll
    if (msg->axes[3] > 0) {
        joint_positions[5] += 0.01;
        joint_positions[6] -= 0.01;
    } else if (msg->axes[3] < 0) {
        joint_positions[5] -= 0.01;
        joint_positions[6] += 0.01;
    }

    auto traj = trajectory_msgs::msg::JointTrajectory();
    traj.header.stamp = this->get_clock()->now();
    traj.joint_names = joint_names;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    traj.points.push_back(point);
    trajectory_pub->publish(traj);

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}