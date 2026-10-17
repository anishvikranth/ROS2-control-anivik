#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <vector>
#include <string>

// Real-hardware teleop node, replacing both sim_control_node.cpp (for real
// hardware use) and arm_control_node_v2.py entirely.
//
// Control modes:
//   MANUAL         - joystick moves target joint POSITIONS, sent to arm_controller
//                    (JointTrajectoryController). Needs encoders.
//   JOYSTICK_ONLY  - joystick sends joint VELOCITIES straight to
//                    joystick_only_controller (no position loop). Works without
//                    encoders when the hardware runs those motors open loop.
//   AUTONOMOUS     - MoveIt owns arm_controller; this node stays quiet.

// JOYSTICK CONTROLLER MAPPING TABLE ( Update here also!!! after changing the controller or its mapping in /joy_arm )

// --- AXIS MAPPING (Joint Movements) ---
// | Axis Index | Joint Target            | ROS Joint Name           | Motion Type       |
// |------------|-------------------------|--------------------------|-------------------|
// | axes[0]    | Base Joint              | base_link_joint          | Continuous (+/-)  |
// | axes[1]    | Shoulder Joint          | shoulder_link_joint      | Continuous (+/-)  |
// | axes[4]    | Elbow Joint             | elbow_link_joint         | Continuous (+/-)  |
// | axes[7]    | Wrist Pitch Joint       | wrist_link_joint         | Continuous (+/-)  |
// | axes[6]    | Palm Roll Joint         | palm_link_joint          | Continuous (+/-)  |
// | axes[3]    | Left/Right Finger Joints| left/right_finger_joint  | Discrete Open/Close|




// --- BUTTON MAPPING (State & Speed Control) ---
// | Button Index | Action / Feature                                                  |
// |--------------|-------------------------------------------------------------------|
// (Toggles fire once per press; holding a button does not repeat.)
// | button[0]    | Toggle Autonomous / Manual mode (Publish /arm_state)              |
// | button[1]    | Toggle Joint-Space / Task-Space jogging (Publish /input_space)    |
// | button[2]    | Toggle Joystick-only (open-loop velocity) mode  [param]           |
// | button[6]    | Step Down k_level (Decrease Sensitivity / Speed)                  |
// | button[7]    | Step Up k_level (Increase Sensitivity / Speed)                    |
//
// --- MODIFIERS (MANUAL mode; while held, the buttons above do NOT do their normal job) ---
// | LB (4) + A/B/X/Y | Move arm to named pose stow / panel_ready / cache_drop / ground_pick |
// | LB (4) + Start(7)| Print the current arm pose as an SRDF <group_state> block           |
// | RB (5) + A / B   | Gripper open / closed (SRDF gripper states)                          |
// Any arm stick cancels a pose move; the gripper stick cancels a gripper preset.
// Pose values come from the SRDF (passed in by arm_real.launch.py).
//
// --- TASK-SPACE JOGGING (button B in MANUAL mode) -- sticks move the GRIPPER, not joints ---
// | axes[1] left stick up/down    | gripper forward / back (along the arm's heading)   |
// | axes[0] left stick left/right | gripper left / right                               |
// | axes[4] right stick up/down   | gripper up / down                                  |
// | axes[7] d-pad up/down         | gripper pitch (tilt up / down)                     |
// | axes[6] d-pad left/right      | gripper roll                                       |
// Directions not being commanded are held, e.g. pushing forward keeps the gripper level.
// The 5-joint arm can't yaw the gripper independently, so yaw is not controlled.

// Highest axis/button index this node reads. Used to bounds-check incoming
// /joy_arm messages before touching them -- see joy_callback. Bump these if
// the mapping table above ever grows to use a higher index.
static constexpr size_t kMinRequiredAxes = 8;    // indices 0..7 used (axes[7] is highest)
static constexpr size_t kMinRequiredButtons = 8; // indices 0..7 used (button[7] is highest)

class RealArmTeleopNode : public rclcpp::Node {
public:
    RealArmTeleopNode() : rclcpp::Node("real_arm_teleop_node") {
        this->declare_parameter<double>("time_from_start_s", 0.15);
        this->declare_parameter<double>("cooldown_s", 0.3);
        this->declare_parameter<int>("joystick_only_button_idx", 2);
        this->declare_parameter<bool>("start_joystick_only", false);
        this->declare_parameter<double>("joy_timeout_s", 0.5);
        this->declare_parameter<std::vector<double>>("velocity_levels", {0.1, 0.25, 0.5});
        this->declare_parameter<int>("pose_modifier_button", 4);
        this->declare_parameter<int>("gripper_modifier_button", 5);
        this->declare_parameter<int>("pose_record_button", 7);
        // Pose for face buttons A, B, X, Y while the pose modifier is held.
        this->declare_parameter<std::vector<std::string>>(
            "pose_buttons", {"stow", "panel_ready", "cache_drop", "ground_pick"});
        // Filled from the SRDF by the launch file: names + 5 values each (arm joint order).
        this->declare_parameter<std::vector<std::string>>("pose_names", std::vector<std::string>{});
        this->declare_parameter<std::vector<double>>("pose_values", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("gripper_open", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("gripper_closed", std::vector<double>{});
        this->declare_parameter<double>("pose_speed", 0.3);     // rad/s of the joint that moves most
        this->declare_parameter<double>("gripper_speed", 0.5);  // rad/s
        // Task-space jogging. robot_description(_semantic) come from the launch file.
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
        this->declare_parameter<std::vector<double>>("tool_offset", {0.0, 0.17, 0.0});  // tool point in palm frame (m)
        this->declare_parameter<std::vector<double>>("task_linear_levels", {0.02, 0.05, 0.10});   // m/s per speed tier
        this->declare_parameter<std::vector<double>>("task_angular_levels", {0.10, 0.25, 0.50});  // rad/s per tier
        this->declare_parameter<double>("task_min_z", -1.0e9);  // keep the tool point above this height (world z)
        this->declare_parameter<double>("task_damping", 0.03);  // damped least squares: larger = calmer near singularities
        this->declare_parameter<std::vector<double>>("task_max_joint_speed", {1.0, 1.0, 1.0, 0.5, 0.5});  // rad/s
        time_from_start_s_ = this->get_parameter("time_from_start_s").as_double();
        cooldown_s_ = this->get_parameter("cooldown_s").as_double();
        joystick_only_button_idx_ =
            static_cast<size_t>(this->get_parameter("joystick_only_button_idx").as_int());
        joy_timeout_s_ = this->get_parameter("joy_timeout_s").as_double();
        velocity_levels = this->get_parameter("velocity_levels").as_double_array();
        // joystick_only_ mirrors which controller is active. The launch file
        // spawns joystick_only_controller active when start_joystick_only is set.
        joystick_only_ = this->get_parameter("start_joystick_only").as_bool();
        pose_modifier_button_ = static_cast<size_t>(this->get_parameter("pose_modifier_button").as_int());
        gripper_modifier_button_ = static_cast<size_t>(this->get_parameter("gripper_modifier_button").as_int());
        pose_record_button_ = static_cast<size_t>(this->get_parameter("pose_record_button").as_int());
        pose_buttons_ = this->get_parameter("pose_buttons").as_string_array();
        pose_speed_ = this->get_parameter("pose_speed").as_double();
        gripper_speed_ = this->get_parameter("gripper_speed").as_double();
        load_poses();

        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy_arm", 10,
            std::bind(&RealArmTeleopNode::joy_callback, this, std::placeholders::_1));
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RealArmTeleopNode::joint_state_callback, this, std::placeholders::_1));

        trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        gripper_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/gripper_controller/joint_trajectory", 10);
        velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joystick_only_controller/commands", 10);
        arm_state_pub = this->create_publisher<std_msgs::msg::Bool>("/arm_state", 10);
        input_space_pub = this->create_publisher<std_msgs::msg::Bool>("/input_space", 10);
        mode_pub = this->create_publisher<std_msgs::msg::String>("/arm_control_mode", 10);
        // MoveIt's RViz panel keeps its own start/goal state, which is NOT refreshed
        // while the joystick moves the arm. Without these, "Plan" right after
        // switching to AUTONOMOUS plans from a stale pose and execution fails.
        rviz_start_state_pub = this->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_start_state", 10);
        rviz_goal_state_pub = this->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_goal_state", 10);

        switch_client = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

        // Joystick-only velocities are streamed from a timer (not from
        // joy_callback) so that a dropped radio link turns into zero velocity
        // after joy_timeout_s. JointGroupVelocityController itself has no
        // timeout and would otherwise keep the last velocity forever.
        velocity_timer = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&RealArmTeleopNode::velocity_timer_callback, this));
        mode_timer = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&RealArmTeleopNode::publish_mode, this));

        joint_names = {
            "base_link_joint",
            "shoulder_link_joint",
            "elbow_link_joint",
            "wrist_link_joint",
            "palm_link_joint",
            "left_finger_link_joint",
            "right_finger_link_joint"
        };

        // Real starting targets come from /joint_states (see resync_targets);
        // nothing is sent to arm_controller until the first one arrives.
        joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // From rover_arm_real.xacro's <limit> tags. Update here if the arm's
        // physical limits ever change -- nothing keeps these in sync with
        // the xacro automatically.
        lower_limits = {-1.570795, -0.3927, -3.141593, -1.570795, -3.141593, -1.570795, -0.785398};
        upper_limits = { 1.570795,  1.5708,  3.141593,  1.570795,  3.141593,  0.785398,  1.570795};

        // Step-size tiers -- replaces v2's pwm_array = {127, 180, 255}.
        // Start conservative; widen once you've confirmed real motor
        // response at the smallest tier. velocity_levels (rad/s) are the
        // joystick-only equivalents and share the same tier index.
        k_levels = {0.005, 0.01, 0.02};
        if (velocity_levels.size() != k_levels.size()) {
            RCLCPP_WARN(this->get_logger(), "velocity_levels must have %zu entries; using defaults.", k_levels.size());
            velocity_levels = {0.1, 0.25, 0.5};
        }
        k_index = 0;
        init_kinematics();  // after k_levels (its tier lists must match their size)

        last_state_toggle_time = this->now();
        last_space_toggle_time = this->now();
        last_joystick_only_toggle_time = this->now();

        RCLCPP_INFO(this->get_logger(), "RealArmTeleopNode initialized in %s mode. Initial k = %.3f",
                    joystick_only_ ? "JOYSTICK_ONLY" : "MANUAL", k_levels[k_index]);
        RCLCPP_INFO(this->get_logger(),
                    "Params: time_from_start_s=%.2f cooldown_s=%.2f joy_timeout_s=%.2f "
                    "joystick_only_button_idx=%zu velocity_levels=[%.2f, %.2f, %.2f] rad/s",
                    time_from_start_s_, cooldown_s_, joy_timeout_s_, joystick_only_button_idx_,
                    velocity_levels[0], velocity_levels[1], velocity_levels[2]);
        RCLCPP_INFO(this->get_logger(), "Waiting for /joy_arm and /joint_states...");
    }

private:
    static constexpr int autonomous_button_idx = 0;   // Button 'A' -- matches v2
    static constexpr int space_toggle_button_idx = 1; // Button 'B' -- matches v2
    static constexpr int mode_down_button_idx = 6;    // matches v2
    static constexpr int mode_up_button_idx = 7;      // matches v2

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr input_space_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rviz_start_state_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rviz_goal_state_pub;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client;
    rclcpp::TimerBase::SharedPtr velocity_timer;
    rclcpp::TimerBase::SharedPtr mode_timer;

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    std::vector<double> k_levels;
    std::vector<double> velocity_levels;
    size_t k_index;

    double time_from_start_s_;
    double cooldown_s_;
    double joy_timeout_s_;
    size_t joystick_only_button_idx_;

    bool arm_state_ = false;     // False = manual, True = autonomous
    bool input_space_ = false;   // published for parity; not acted on here
    bool joystick_only_ = false; // True = joystick_only_controller is active
    bool switch_pending_ = false;
    bool need_resync_ = true;    // reload joint_positions from /joint_states before next trajectory
    rclcpp::Time last_state_toggle_time;
    rclcpp::Time last_space_toggle_time;
    rclcpp::Time last_joystick_only_toggle_time;
    bool prev_mode_up_btn_ = false;
    bool prev_mode_down_btn_ = false;
    std::vector<int32_t> prev_buttons_;  // for press (rising-edge) detection

    // Named poses / gripper presets (MANUAL mode)
    static constexpr size_t kArmJoints = 5;  // joint_names[0..4]; [5..6] are the fingers
    size_t pose_modifier_button_ = 4;
    size_t gripper_modifier_button_ = 5;
    size_t pose_record_button_ = 7;
    std::vector<std::string> pose_buttons_;
    std::unordered_map<std::string, std::vector<double>> poses_;
    std::vector<double> gripper_open_, gripper_closed_;
    double pose_speed_ = 0.3;
    double gripper_speed_ = 0.5;
    bool pose_move_active_ = false;
    std::string pose_move_name_;
    rclcpp::Time pose_move_end_{0, 0, RCL_ROS_TIME};
    bool gripper_move_active_ = false;
    rclcpp::Time gripper_move_end_{0, 0, RCL_ROS_TIME};
    static constexpr double kStickCancel = 0.1;  // |axis| that cancels a pose / gripper move

    // Task-space jogging
    moveit::core::RobotModelPtr robot_model_;
    std::unique_ptr<moveit::core::RobotState> kin_state_;
    const moveit::core::JointModelGroup * arm_jmg_ = nullptr;
    const moveit::core::LinkModel * palm_link_ = nullptr;
    Eigen::Vector3d tool_offset_{0.0, 0.17, 0.0};
    std::vector<double> task_linear_levels_, task_angular_levels_, task_max_joint_speed_;
    double task_min_z_ = -1.0e9;
    double task_damping_ = 0.03;
    rclcpp::Time last_task_time_{0, 0, RCL_ROS_TIME};
    bool joy_lost_ = false;           // edge-triggered "joystick lost" logging
    rclcpp::Time last_joy_lost_log_{0, 0, RCL_ROS_TIME};
    bool joint_states_seen_ = false;
    rclcpp::Time switch_request_time_{0, 0, RCL_ROS_TIME};
    static constexpr double kSwitchTimeoutS = 3.0;
    // After entering AUTONOMOUS, refresh RViz a few times while the last
    // joystick trajectory (time_from_start_s) settles.
    rclcpp::Time rviz_sync_until_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_rviz_sync_{0, 0, RCL_ROS_TIME};
    static constexpr double kRvizSyncWindowS = 1.0;
    static constexpr double kRvizSyncPeriodS = 0.25;

    sensor_msgs::msg::Joy::SharedPtr last_joy_;
    rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
    std::unordered_map<std::string, double> measured_positions_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void velocity_timer_callback();
    void load_poses();
    void init_kinematics();
    void task_space_step(const sensor_msgs::msg::Joy & msg, const rclcpp::Time & now);
    bool start_pose_move(const std::string & name, const rclcpp::Time & now);
    void start_gripper_move(const std::vector<double> & target, const char * label, const rclcpp::Time & now);
    void print_current_pose();
    void publish_arm(const rclcpp::Time & now, double duration_s);
    void publish_gripper(const rclcpp::Time & now, double duration_s);
    void check_joy_link();
    void publish_mode();
    void request_joystick_only(bool enable);
    bool resync_targets();
};

void RealArmTeleopNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!joint_states_seen_) {
        joint_states_seen_ = true;
        RCLCPP_INFO(this->get_logger(), "Receiving /joint_states (%zu joints).", msg->name.size());
    }
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        measured_positions_[msg->name[i]] = msg->position[i];
    }
}

// Copy the arm's measured pose into joint_positions so the next trajectory
// starts from where the arm actually is, not from a stale target.
bool RealArmTeleopNode::resync_targets() {
    for (const auto & name : joint_names) {
        if (measured_positions_.find(name) == measured_positions_.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Cannot sync joystick targets: joint '%s' missing from /joint_states.", name.c_str());
            return false;
        }
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
        joint_positions[i] = std::clamp(measured_positions_[joint_names[i]], lower_limits[i], upper_limits[i]);
    }
    need_resync_ = false;
    RCLCPP_INFO(this->get_logger(), "Joystick targets synced to current arm pose.");
    return true;
}

void RealArmTeleopNode::publish_mode() {
    std_msgs::msg::String msg;
    msg.data = arm_state_ ? "AUTONOMOUS" : (joystick_only_ ? "JOYSTICK_ONLY" : "MANUAL");
    mode_pub->publish(msg);
}

void RealArmTeleopNode::request_joystick_only(bool enable) {
    if (!switch_client->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "controller_manager switch service not available; mode unchanged.");
        return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    const std::vector<std::string> position_controllers = {"arm_controller", "gripper_controller"};
    const std::vector<std::string> velocity_controller = {"joystick_only_controller"};
    request->activate_controllers = enable ? velocity_controller : position_controllers;
    request->deactivate_controllers = enable ? position_controllers : velocity_controller;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout = rclcpp::Duration::from_seconds(2.0);

    if (!enable) {
        // Stop before handing over; arm_controller will hold the current pose.
        std_msgs::msg::Float64MultiArray zero;
        zero.data.assign(joint_names.size(), 0.0);
        velocity_pub->publish(zero);
    }

    auto join = [](const std::vector<std::string> & v) {
        std::string out;
        for (const auto & n : v) out += (out.empty() ? "" : " + ") + n;
        return out;
    };
    RCLCPP_INFO(this->get_logger(), "Requesting controller switch: activate %s, deactivate %s...",
                join(request->activate_controllers).c_str(), join(request->deactivate_controllers).c_str());
    pose_move_active_ = gripper_move_active_ = false;
    switch_pending_ = true;
    switch_request_time_ = this->now();
    switch_client->async_send_request(
        request,
        [this, enable](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
            if (!switch_pending_) {
                RCLCPP_WARN(this->get_logger(), "Late controller switch response (after timeout) -- applying it now.");
            }
            switch_pending_ = false;
            const auto response = future.get();
            if (!response->ok) {
                RCLCPP_ERROR(this->get_logger(), "Controller switch failed: %s", response->message.c_str());
                return;
            }
            joystick_only_ = enable;
            if (!enable) {
                need_resync_ = true;
            }
            RCLCPP_INFO(this->get_logger(), "Switched Arm Mode to: %s",
                        enable ? "JOYSTICK_ONLY (velocity, no position loop)" : "MANUAL (position, closed loop)");
            publish_mode();
        });
}

// Edge-triggered joystick link monitoring (all modes).
void RealArmTeleopNode::check_joy_link() {
    if (!last_joy_) {
        return;
    }
    const double age = (this->now() - last_joy_time_).seconds();
    if (age >= joy_timeout_s_ && joy_lost_ && joystick_only_ && !arm_state_ &&
        (this->now() - last_joy_lost_log_).seconds() >= 5.0) {
        last_joy_lost_log_ = this->now();
        RCLCPP_ERROR(this->get_logger(), "Joystick still lost (%.1f s) -- joint velocities held at 0.", age);
    } else if (age >= joy_timeout_s_ && !joy_lost_) {
        joy_lost_ = true;
        last_joy_lost_log_ = this->now();
        if (arm_state_) {
            RCLCPP_INFO(this->get_logger(), "Joystick signal lost (%.2f s) -- AUTONOMOUS mode, MoveIt keeps control.", age);
        } else if (joystick_only_) {
            RCLCPP_ERROR(this->get_logger(), "JOYSTICK SIGNAL LOST (no /joy_arm for %.2f s) -- setting all joint velocities = 0 (PWM -> 0).", age);
        } else {
            RCLCPP_WARN(this->get_logger(), "JOYSTICK SIGNAL LOST (no /joy_arm for %.2f s) -- arm_controller holds the last target.", age);
        }
    } else if (age < joy_timeout_s_ && joy_lost_) {
        joy_lost_ = false;
        RCLCPP_INFO(this->get_logger(), "Joystick signal restored.");
    }
}

void RealArmTeleopNode::velocity_timer_callback() {
    check_joy_link();

    const rclcpp::Time now = this->now();
    if (arm_state_ && now < rviz_sync_until_ && (now - last_rviz_sync_).seconds() >= kRvizSyncPeriodS) {
        last_rviz_sync_ = now;
        rviz_start_state_pub->publish(std_msgs::msg::Empty());
        rviz_goal_state_pub->publish(std_msgs::msg::Empty());
    }

    if (switch_pending_ && (this->now() - switch_request_time_).seconds() > kSwitchTimeoutS) {
        switch_pending_ = false;
        RCLCPP_ERROR(this->get_logger(), "Controller switch got no response in %.1f s -- mode unchanged (%s). "
                     "Is controller_manager alive?", kSwitchTimeoutS, joystick_only_ ? "JOYSTICK_ONLY" : "MANUAL");
    }

    if (!joystick_only_ || arm_state_ || switch_pending_) {
        return;
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(joint_names.size(), 0.0);

    const bool joy_fresh = last_joy_ && (this->now() - last_joy_time_).seconds() < joy_timeout_s_;
    if (!joy_fresh) {
        velocity_pub->publish(cmd);  // lost/restored logging is in check_joy_link()
        return;
    }

    const auto & axes = last_joy_->axes;
    const double v = velocity_levels[k_index];
    cmd.data[0] = v * axes[0];     // Base
    cmd.data[1] = v * axes[1];     // Shoulder
    cmd.data[2] = v * axes[4];     // Elbow
    cmd.data[3] = v * axes[7];     // Wrist (pitch)
    cmd.data[4] = v * axes[6];     // Palm (roll)
    if (axes[3] > 0) {
        cmd.data[5] = v;
        cmd.data[6] = -v;
    } else if (axes[3] < 0) {
        cmd.data[5] = -v;
        cmd.data[6] = v;
    }

    // Soft limits from /joint_states. With open-loop motors these positions
    // are dead-reckoned estimates, so this is a best-effort guard only --
    // hard stops / limit switches are still needed.
    for (size_t i = 0; i < joint_names.size(); ++i) {
        const auto it = measured_positions_.find(joint_names[i]);
        if (it == measured_positions_.end()) {
            continue;
        }
        if ((it->second >= upper_limits[i] && cmd.data[i] > 0.0) ||
            (it->second <= lower_limits[i] && cmd.data[i] < 0.0)) {
            cmd.data[i] = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Joint '%s' at its (estimated) limit; velocity zeroed.", joint_names[i].c_str());
        }
    }

    velocity_pub->publish(cmd);
}

void RealArmTeleopNode::load_poses() {
    const auto names = this->get_parameter("pose_names").as_string_array();
    const auto values = this->get_parameter("pose_values").as_double_array();
    if (values.size() != names.size() * kArmJoints) {
        RCLCPP_WARN(this->get_logger(), "pose_names/pose_values mismatch (%zu names, %zu values) -- named poses disabled.",
                    names.size(), values.size());
    } else {
        for (size_t i = 0; i < names.size(); ++i) {
            poses_[names[i]] = std::vector<double>(values.begin() + i * kArmJoints, values.begin() + (i + 1) * kArmJoints);
        }
    }
    gripper_open_ = this->get_parameter("gripper_open").as_double_array();
    gripper_closed_ = this->get_parameter("gripper_closed").as_double_array();
    std::string list;
    for (size_t i = 0; i < pose_buttons_.size() && i < 4; ++i) {
        static const char * face[] = {"A", "B", "X", "Y"};
        list += std::string(list.empty() ? "" : ", ") + "LB+" + face[i] + "=" + pose_buttons_[i] +
                (poses_.count(pose_buttons_[i]) ? "" : " (NOT IN SRDF)");
    }
    RCLCPP_INFO(this->get_logger(), "Named poses: %s. Gripper presets: RB+A=open%s, RB+B=closed%s.", list.c_str(),
                gripper_open_.size() == 2 ? "" : " (missing)", gripper_closed_.size() == 2 ? "" : " (missing)");
}

void RealArmTeleopNode::publish_arm(const rclcpp::Time & now, double duration_s) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now;
    traj.joint_names.assign(joint_names.begin(), joint_names.begin() + kArmJoints);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(joint_positions.begin(), joint_positions.begin() + kArmJoints);
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    traj.points.push_back(point);
    trajectory_pub->publish(traj);
}

void RealArmTeleopNode::publish_gripper(const rclcpp::Time & now, double duration_s) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now;
    traj.joint_names.assign(joint_names.begin() + kArmJoints, joint_names.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(joint_positions.begin() + kArmJoints, joint_positions.end());
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    traj.points.push_back(point);
    gripper_pub->publish(traj);
}

bool RealArmTeleopNode::start_pose_move(const std::string & name, const rclcpp::Time & now) {
    const auto it = poses_.find(name);
    if (it == poses_.end()) {
        RCLCPP_WARN(this->get_logger(), "Pose '%s' is not defined in the SRDF -- ignored.", name.c_str());
        return false;
    }
    double max_delta = 0.0;
    for (size_t i = 0; i < kArmJoints; ++i) {
        const auto m = measured_positions_.find(joint_names[i]);
        if (m == measured_positions_.end()) {
            RCLCPP_WARN(this->get_logger(), "No /joint_states yet -- can't move to pose '%s'.", name.c_str());
            return false;
        }
        const double target = std::clamp(it->second[i], lower_limits[i], upper_limits[i]);
        max_delta = std::max(max_delta, std::abs(target - m->second));
        joint_positions[i] = target;
    }
    // Joint-space move: every joint arrives together, the furthest one at pose_speed.
    const double duration = std::max(max_delta / std::max(pose_speed_, 0.01), 1.0);
    publish_arm(now, duration);
    pose_move_active_ = true;
    pose_move_name_ = name;
    pose_move_end_ = now + rclcpp::Duration::from_seconds(duration + 0.2);
    RCLCPP_INFO(this->get_logger(), "Moving to pose '%s' (%.1f s). Any arm stick cancels.", name.c_str(), duration);
    return true;
}

void RealArmTeleopNode::start_gripper_move(const std::vector<double> & target, const char * label, const rclcpp::Time & now) {
    if (target.size() != 2) {
        RCLCPP_WARN(this->get_logger(), "Gripper '%s' preset missing from the SRDF -- ignored.", label);
        return;
    }
    double max_delta = 0.0;
    for (size_t k = 0; k < 2; ++k) {
        const size_t i = kArmJoints + k;
        const double t = std::clamp(target[k], lower_limits[i], upper_limits[i]);
        const auto m = measured_positions_.find(joint_names[i]);
        if (m != measured_positions_.end()) {
            max_delta = std::max(max_delta, std::abs(t - m->second));
        }
        joint_positions[i] = t;
    }
    const double duration = std::max(max_delta / std::max(gripper_speed_, 0.01), 0.5);
    publish_gripper(now, duration);
    gripper_move_active_ = true;
    gripper_move_end_ = now + rclcpp::Duration::from_seconds(duration + 0.2);
    RCLCPP_INFO(this->get_logger(), "Gripper -> %s (%.1f s). Gripper stick cancels.", label, duration);
}

void RealArmTeleopNode::print_current_pose() {
    std::string block = "Current arm pose -- paste into rover_arm.srdf and rename:\n"
                        "    <group_state name=\"NEW_POSE\" group=\"arm\">\n";
    for (size_t i = 0; i < kArmJoints; ++i) {
        const auto m = measured_positions_.find(joint_names[i]);
        if (m == measured_positions_.end()) {
            RCLCPP_WARN(this->get_logger(), "No /joint_states yet -- can't print the pose.");
            return;
        }
        char line[128];
        std::snprintf(line, sizeof(line), "        <joint name=\"%s\" value=\"%.3f\"/>\n", joint_names[i].c_str(), m->second);
        block += line;
    }
    block += "    </group_state>";
    RCLCPP_INFO(this->get_logger(), "%s", block.c_str());
}

void RealArmTeleopNode::init_kinematics() {
    const auto urdf_xml = this->get_parameter("robot_description").as_string();
    const auto srdf_xml = this->get_parameter("robot_description_semantic").as_string();
    auto vec = [this](const char * name) { return this->get_parameter(name).as_double_array(); };
    task_linear_levels_ = vec("task_linear_levels");
    task_angular_levels_ = vec("task_angular_levels");
    task_max_joint_speed_ = vec("task_max_joint_speed");
    task_min_z_ = this->get_parameter("task_min_z").as_double();
    task_damping_ = this->get_parameter("task_damping").as_double();
    const auto off = vec("tool_offset");
    if (off.size() == 3) tool_offset_ = Eigen::Vector3d(off[0], off[1], off[2]);
    if (task_linear_levels_.size() != k_levels.size() || task_angular_levels_.size() != k_levels.size() ||
        task_max_joint_speed_.size() != kArmJoints) {
        RCLCPP_WARN(this->get_logger(), "task_* parameter sizes are wrong -- using defaults.");
        task_linear_levels_ = {0.02, 0.05, 0.10};
        task_angular_levels_ = {0.10, 0.25, 0.50};
        task_max_joint_speed_ = {1.0, 1.0, 1.0, 0.5, 0.5};
    }
    if (urdf_xml.empty() || srdf_xml.empty()) {
        RCLCPP_WARN(this->get_logger(), "No robot_description(_semantic) parameter -- task-space jogging disabled.");
        return;
    }
    auto urdf_model = std::make_shared<urdf::Model>();
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!urdf_model->initString(urdf_xml) || !srdf_model->initString(*urdf_model, srdf_xml)) {
        RCLCPP_WARN(this->get_logger(), "Could not parse the URDF/SRDF -- task-space jogging disabled.");
        return;
    }
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    arm_jmg_ = robot_model_->getJointModelGroup("arm");
    palm_link_ = robot_model_->getLinkModel("palm_link");
    if (!arm_jmg_ || !palm_link_ || arm_jmg_->getVariableCount() != kArmJoints) {
        RCLCPP_WARN(this->get_logger(), "SRDF has no 5-joint 'arm' group / palm_link -- task-space jogging disabled.");
        arm_jmg_ = nullptr;
        return;
    }
    kin_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    kin_state_->setToDefaultValues();
    RCLCPP_INFO(this->get_logger(), "Task-space jogging ready (button B). Tool point %.2f m ahead of the palm.",
                tool_offset_.norm());
}

// One joystick step of task-space jogging: stick command (forward, left, up, pitch,
// roll) -> joint velocities through the 5x5 task Jacobian -> new joint targets.
void RealArmTeleopNode::task_space_step(const sensor_msgs::msg::Joy & msg, const rclcpp::Time & now) {
    const double dt = std::clamp((now - last_task_time_).seconds(), 0.0, 0.1);
    last_task_time_ = now;
    if (dt <= 0.0) return;

    const double lin = task_linear_levels_[k_index];
    const double ang = task_angular_levels_[k_index];
    double fwd_cmd = msg.axes[1], left_cmd = msg.axes[0], up_cmd = msg.axes[4];
    const double pitch_cmd = msg.axes[7], roll_cmd = msg.axes[6];
    if (std::abs(fwd_cmd) + std::abs(left_cmd) + std::abs(up_cmd) + std::abs(pitch_cmd) + std::abs(roll_cmd) < 1e-3) {
        return;  // sticks centred: hold
    }

    Eigen::VectorXd q(kArmJoints);
    for (size_t i = 0; i < kArmJoints; ++i) q[i] = joint_positions[i];
    kin_state_->setJointGroupPositions(arm_jmg_, q);
    kin_state_->update();
    Eigen::MatrixXd jac;  // 6 x 5: linear xyz, angular xyz, in the world frame
    if (!kin_state_->getJacobian(arm_jmg_, palm_link_, tool_offset_, jac)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Jacobian failed -- task-space step skipped.");
        return;
    }
    const Eigen::Isometry3d & palm = kin_state_->getGlobalLinkTransform(palm_link_);
    const Eigen::Vector3d tool = palm * tool_offset_;

    // Directions: "forward" follows the arm's heading (base joint), up is world up.
    const double th = q[0];
    const Eigen::Vector3d fwd(std::sin(th), -std::cos(th), 0.0);
    const Eigen::Vector3d up(0.0, 0.0, 1.0);
    const Eigen::Vector3d left = up.cross(fwd);
    const Eigen::Vector3d tool_axis = palm.rotation().col(1);  // palm +y = approach / roll axis

    // Floor: no further downward motion once the tool point reaches task_min_z.
    if (tool.z() <= task_min_z_ && up_cmd < 0.0) {
        up_cmd = 0.0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Task space: tool at the floor limit (z = %.2f m) -- down blocked.", tool.z());
    }

    Eigen::Matrix<double, 5, 5> jt;
    jt.row(0) = fwd.transpose() * jac.topRows(3);
    jt.row(1) = left.transpose() * jac.topRows(3);
    jt.row(2) = up.transpose() * jac.topRows(3);
    jt.row(3) = -left.transpose() * jac.bottomRows(3);   // pitch up = nose up
    jt.row(4) = tool_axis.transpose() * jac.bottomRows(3);
    Eigen::Matrix<double, 5, 1> v;
    v << fwd_cmd * lin, left_cmd * lin, up_cmd * lin, pitch_cmd * ang, roll_cmd * ang;

    // Damped least squares: exact away from singularities, slows down near them.
    const Eigen::Matrix<double, 5, 5> jjt =
        jt * jt.transpose() + task_damping_ * task_damping_ * Eigen::Matrix<double, 5, 5>::Identity();
    Eigen::Matrix<double, 5, 1> qdot = jt.transpose() * jjt.ldlt().solve(v);

    // Near a singularity (e.g. arm stretched straight) part of the command can't be
    // done, and damped least squares then moves the gripper in a direction that wasn't
    // asked for. Measure how much of the command is achievable and slow down by the rest.
    const double off = (jt * qdot - v).norm() / std::max(v.norm(), 1e-9);
    if (off > 0.3) {
        qdot *= std::max(0.0, 1.0 - off);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                             "Task space: can't move that way from here (arm stretched or at a limit) -- motion reduced.");
    }

    // Respect joint speed limits by scaling the whole step (keeps the direction).
    double scale = 1.0;
    for (size_t i = 0; i < kArmJoints; ++i) {
        if (std::abs(qdot[i]) > task_max_joint_speed_[i]) scale = std::min(scale, task_max_joint_speed_[i] / std::abs(qdot[i]));
    }
    if (scale < 0.5) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Task space: near a singularity / joint speed limit -- motion slowed to %.0f%%.", scale * 100.0);
    }
    for (size_t i = 0; i < kArmJoints; ++i) joint_positions[i] += scale * qdot[i] * dt;
}

void RealArmTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Guard against malformed/short Joy messages -- msg->axes[...] and
    // msg->buttons[...] below use operator[], which does NOT bounds-check.
    // A joystick driver that publishes fewer axes/buttons than we expect
    // (wrong controller, driver hiccup, bad remap) would otherwise be
    // undefined behavior -- on real hardware that can mean anything from a
    // crash to silently commanding garbage joint positions. Reject the
    // message instead and wait for a well-formed one.
    const size_t required_buttons = std::max(kMinRequiredButtons, joystick_only_button_idx_ + 1);
    if (msg->axes.size() < kMinRequiredAxes || msg->buttons.size() < required_buttons) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring /joy_arm message: got %zu axes / %zu buttons, need at least %zu / %zu. "
            "Check the joystick mapping table at the top of this file against 'ros2 topic echo /joy_arm'.",
            msg->axes.size(), msg->buttons.size(), kMinRequiredAxes, required_buttons);
        return;
    }

    const rclcpp::Time now = this->now();
    if (!last_joy_) {
        RCLCPP_INFO(this->get_logger(), "Joystick connected: %zu axes, %zu buttons.", msg->axes.size(), msg->buttons.size());
    }
    last_joy_ = msg;
    last_joy_time_ = now;

    // Mode toggles fire once per PRESS (rising edge), plus the cooldown as
    // debounce. Previously a held button re-toggled every cooldown_s, so
    // holding A flipped in and out of AUTONOMOUS.
    auto pressed = [&](size_t idx) {
        return msg->buttons[idx] && !(idx < prev_buttons_.size() && prev_buttons_[idx]);
    };
    auto held = [&](size_t idx) { return idx < msg->buttons.size() && msg->buttons[idx]; };
    const bool pose_mod = held(pose_modifier_button_);
    const bool grip_mod = held(gripper_modifier_button_);
    const bool modifier = pose_mod || grip_mod;
    bool face_pressed[4];
    for (size_t i = 0; i < 4; ++i) face_pressed[i] = pressed(i);
    const bool record_pressed = pose_record_button_ < msg->buttons.size() && pressed(pose_record_button_);
    // While LB/RB is held, A/B/X/Y/Start select poses instead of their normal job.
    const bool autonomous_pressed = !modifier && pressed(autonomous_button_idx);
    const bool joystick_only_pressed = !modifier && pressed(joystick_only_button_idx_);
    const bool space_pressed = !modifier && pressed(space_toggle_button_idx);
    prev_buttons_ = msg->buttons;

    // arm_state is published every callback regardless of mode -- this is
    // the single source of truth other nodes read.
    std_msgs::msg::Bool state_msg;
    state_msg.data = arm_state_;
    arm_state_pub->publish(state_msg);

    // AUTONOMOUS TOGGLE (button A), debounced.
    if (autonomous_pressed &&
        (now - last_state_toggle_time).seconds() > cooldown_s_) {
        last_state_toggle_time = now;
        if (joystick_only_ || switch_pending_) {
            // MoveIt drives arm_controller, which isn't active in joystick-only mode.
            RCLCPP_WARN(this->get_logger(), "Leave JOYSTICK_ONLY mode before switching to AUTONOMOUS.");
        } else {
            arm_state_ = !arm_state_;
            if (!arm_state_) {
                need_resync_ = true; // MoveIt moved the arm; don't jump back to old targets
            } else {
                // Point RViz's start + goal state at the arm's real pose.
                rviz_sync_until_ = now + rclcpp::Duration::from_seconds(kRvizSyncWindowS);
                last_rviz_sync_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                RCLCPP_INFO(this->get_logger(), "RViz MotionPlanning start/goal state synced to the current arm pose.");
            }
            RCLCPP_INFO(this->get_logger(), "Switched Arm Mode to: %s",
                        arm_state_ ? "AUTONOMOUS (MoveIt in control)" : "MANUAL (Joystick in control)");
            publish_mode();
        }
    }

    if (arm_state_) {
        // Autonomous mode: MoveIt owns /arm_controller/joint_trajectory now.
        // Stay completely quiet -- don't compute or publish anything.
        return;
    }

    // JOYSTICK-ONLY TOGGLE, debounced.
    if (joystick_only_pressed && switch_pending_) {
        RCLCPP_WARN(this->get_logger(), "Controller switch already in progress -- button ignored.");
    }
    if (joystick_only_pressed && !switch_pending_ &&
        (now - last_joystick_only_toggle_time).seconds() > cooldown_s_) {
        last_joystick_only_toggle_time = now;
        request_joystick_only(!joystick_only_);
    }

    // INPUT SPACE TOGGLE (button B), debounced. Published for parity/future use.
    if (space_pressed &&
        (now - last_space_toggle_time).seconds() > cooldown_s_) {
        last_space_toggle_time = now;
        if (!input_space_ && !arm_jmg_) {
            RCLCPP_WARN(this->get_logger(), "Task-space jogging unavailable (no robot model) -- staying in JOINT SPACE.");
        } else {
            input_space_ = !input_space_;
            last_task_time_ = now;
            need_resync_ = true;  // start the jog from where the arm really is
        }

        RCLCPP_INFO(this->get_logger(), "Switched Input Space to: %s",
                    input_space_ ? "TASK SPACE (sticks move the gripper)" : "JOINT SPACE (sticks move joints)");
    }
    std_msgs::msg::Bool space_msg;
    space_msg.data = input_space_;
    input_space_pub->publish(space_msg);

    // MODE UP/DOWN (buttons 7/6) -- step-size tier select, edge-triggered.
    // Not while a modifier is held (LB + Start prints the pose instead).
    if (modifier) {
        prev_mode_down_btn_ = msg->buttons[mode_down_button_idx];
        prev_mode_up_btn_ = msg->buttons[mode_up_button_idx];
    } else if (msg->buttons[mode_down_button_idx]) {
        if (!prev_mode_down_btn_) {
            if (k_index > 0) {
                k_index--;
                RCLCPP_INFO(this->get_logger(), "Decreased speed tier to index %zu: k = %.3f rad/step, v = %.2f rad/s",
                            k_index, k_levels[k_index], velocity_levels[k_index]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Already at minimum k_level tier (k = %.3f)", k_levels[k_index]);
            }
            prev_mode_down_btn_ = true;
        }
    } else {
        prev_mode_down_btn_ = false;
    }

    if (modifier) {
        // handled above
    } else if (msg->buttons[mode_up_button_idx]) {
        if (!prev_mode_up_btn_) {
            if (k_index < k_levels.size() - 1) {
                k_index++;
                RCLCPP_INFO(this->get_logger(), "Increased speed tier to index %zu: k = %.3f rad/step, v = %.2f rad/s",
                            k_index, k_levels[k_index], velocity_levels[k_index]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Already at maximum k_level tier (k = %.3f)", k_levels[k_index]);
            }
            prev_mode_up_btn_ = true;
        }
    } else {
        prev_mode_up_btn_ = false;
    }

    const bool any_face = face_pressed[0] || face_pressed[1] || face_pressed[2] || face_pressed[3] || record_pressed;
    if (joystick_only_ || switch_pending_) {
        if (modifier && any_face) {
            RCLCPP_WARN(this->get_logger(), "Named poses / gripper presets need MANUAL mode (closed loop).");
        }
        // Velocities are streamed by velocity_timer_callback.
        return;
    }

    if (need_resync_ && !resync_targets()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Waiting for /joint_states with all arm joints before accepting joystick motion.");
        return;
    }

    // Named poses / gripper presets / pose printout.
    if (pose_mod) {
        for (size_t i = 0; i < 4 && i < pose_buttons_.size(); ++i) {
            if (face_pressed[i]) start_pose_move(pose_buttons_[i], now);
        }
        if (record_pressed) print_current_pose();
    }
    if (grip_mod) {
        if (face_pressed[0]) start_gripper_move(gripper_open_, "open", now);
        if (face_pressed[1]) start_gripper_move(gripper_closed_, "closed", now);
    }

    // A running pose move owns the arm until it ends or an arm stick cancels it.
    const double arm_stick = std::max({std::abs(msg->axes[0]), std::abs(msg->axes[1]), std::abs(msg->axes[4]),
                                       std::abs(msg->axes[6]), std::abs(msg->axes[7])});
    if (pose_move_active_) {
        if (arm_stick > kStickCancel) {
            pose_move_active_ = false;
            resync_targets();  // continue from where the arm actually is
            RCLCPP_WARN(this->get_logger(), "Pose move '%s' CANCELLED by the stick.", pose_move_name_.c_str());
        } else if (now >= pose_move_end_) {
            pose_move_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Reached pose '%s'.", pose_move_name_.c_str());
        }
    }
    if (gripper_move_active_) {
        if (msg->axes[3] != 0.0) {
            gripper_move_active_ = false;
            for (size_t i = kArmJoints; i < joint_names.size(); ++i) {
                const auto m = measured_positions_.find(joint_names[i]);
                if (m != measured_positions_.end()) joint_positions[i] = m->second;
            }
            RCLCPP_WARN(this->get_logger(), "Gripper preset CANCELLED by the stick.");
        } else if (now >= gripper_move_end_) {
            gripper_move_active_ = false;
        }
    }

    const double k = k_levels[k_index];

    // Joint-space motion -- same axis mapping as sim_control_node.cpp
    if (!pose_move_active_ && input_space_ && arm_jmg_) {
        task_space_step(*msg, now);
    } else if (!pose_move_active_) {
        joint_positions[0] += k * msg->axes[0];     // Base
        joint_positions[1] += k * msg->axes[1];     // Shoulder
        joint_positions[2] += k * msg->axes[4];     // Elbow
        joint_positions[3] += k * msg->axes[7];     // Wrist (pitch)
        joint_positions[4] += k * msg->axes[6];     // Palm (roll)
    }
    if (!gripper_move_active_) {
        if (msg->axes[3] > 0) {
            joint_positions[5] += k;
            joint_positions[6] -= k;
        } else if (msg->axes[3] < 0) {
            joint_positions[5] -= k;
            joint_positions[6] += k;
        }
    }

    // Clamp to real joint limits. Also warn (throttled, so a joint held
    // against its limit doesn't spam the console) whenever a clamp actually
    // changes the commanded value -- that's the signal that you just tried
    // to drive a joint past where it's mechanically allowed to go.
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        const double pre_clamp = joint_positions[i];
        joint_positions[i] = std::clamp(joint_positions[i], lower_limits[i], upper_limits[i]);
        if (joint_positions[i] != pre_clamp) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Joint '%s' hit its limit and was clamped to %.3f rad.",
                joint_names[i].c_str(), joint_positions[i]);
        }
    }

    if (!pose_move_active_) publish_arm(now, time_from_start_s_);
    if (!gripper_move_active_) publish_gripper(now, time_from_start_s_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealArmTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
