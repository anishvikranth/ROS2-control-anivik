#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

void PrintRobotInfo(const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& move_group){
    //Print the planning frame
    RCLCPP_INFO(node->get_logger(),"Planning Frame: %s",move_group.getPlanningFrame().c_str());
    //Print the end effector
    RCLCPP_INFO(node->get_logger(),"End effector Link: %s", move_group.getPlanningFrame().c_str());
    //Print all the planning groups
    RCLCPP_INFO(node->get_logger(),"Available Planning Groups:");
    for (const auto &group : move_group.getJointModelGroupNames())
    {
        RCLCPP_INFO(node->get_logger(),"  %s", group.c_str());
    }
}

bool PrintCurrentJointValues(const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& move_group){
    
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
    "moveit_goal_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    PrintRobotInfo(node, move_group);
    RCLCPP_INFO(node->get_logger(), "Sleeping for 2 seconds...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    // Wait up to 5 seconds for the current robot state
    auto current_state = move_group.getCurrentState(5.0);

    if (!current_state)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to get current robot state.");

        rclcpp::shutdown();
        executor.cancel();
        spinner.join();
        return 1;
    }

    // Get the JointModelGroup for the arm
    const moveit::core::JointModelGroup *joint_model_group =
        current_state->getJointModelGroup("arm");

    if (!joint_model_group)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to get JointModelGroup for 'arm'.");

        rclcpp::shutdown();
        executor.cancel();
        spinner.join();
        return 1;
    }

    // Copy the current joint values
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(
        joint_model_group,
        joint_values);

    // Get corresponding joint names
    const std::vector<std::string> &joint_names =
        joint_model_group->getVariableNames();

    // Print joint values
    RCLCPP_INFO(node->get_logger(), "Current Joint Values:");

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(
            node->get_logger(),
            "%s : %f",
            joint_names[i].c_str(),
            joint_values[i]);
    }

    //Setting the start state to current
    RCLCPP_INFO(node->get_logger(), "About to call setStartStateToCurrentState()");
    move_group.setStartStateToCurrentState();
    RCLCPP_INFO(node->get_logger(),
            "Start state updated to current state.");
    RCLCPP_INFO(node->get_logger(), "Returned from setStartStateToCurrentState()");

    //Define the goal state for the arm
    std::vector<double> goal_joint_values =
    {
        1.57,    // base_link_joint
        1.57,    // shoulder_link_joint
        1.57,    // elbow_link_joint
        1.57,    // wrist_link_joint
        1.57,     // palm_link_joint
    };
    move_group.setJointValueTarget(goal_joint_values);
    RCLCPP_INFO(node->get_logger(),"Goal state stored.");

    //Verification 
    std::vector<double> target;
    move_group.getJointValueTarget(target);
    RCLCPP_INFO(node->get_logger(), "Goal Joint Values");
    for (size_t i = 0;i< joint_names.size(); i++){
        RCLCPP_INFO(node->get_logger(),"%s:%f", joint_names[i].c_str(),target[i]);
    }


    //planning trial
    move_group.setStartStateToCurrentState();  

    move_group.setJointValueTarget(goal_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        (move_group.plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(node->get_logger(), "Plan success = %d", success);
    if (success)
    {
        auto result = move_group.execute(plan);

        RCLCPP_INFO(
            node->get_logger(),
            "Execute result = %d",
            result.val);
    }
    rclcpp::shutdown();
    executor.cancel();
    spinner.join();
    return 0;
}