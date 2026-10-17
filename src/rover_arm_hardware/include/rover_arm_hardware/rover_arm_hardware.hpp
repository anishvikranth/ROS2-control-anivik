#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace rover_arm_hardware
{

/**
 * @brief ros2_control Hardware Interface Plugin for the Rover Arm
 *
 * Bridges ros2_control velocity commands and state interfaces to the arm's
 * serial bridge (arm_com_node / arm.py).
 *
 * Responsibilities:
 *  1. Receives raw joint positions (in degrees) from /enc_angles and converts them to radians.
 *  2. Estimates motor velocities from successive /enc_angles messages (low-pass filtered).
 *  3. Translates wrist pitch/roll joint commands into differential motor space (Bevel 1 & 2).
 *  4. Per motor, either:
 *       - CLOSED LOOP: pwm = ff_gain * velocity + PI correction on encoder feedback, or
 *       - OPEN LOOP:   maps velocity straight to PWM (pwm = ff_gain * velocity) and
 *                      reports a dead-reckoned position (integral of the command).
 *     Motors are put in open loop via the "open_loop_joints" hardware parameter.
 *  5. Publishes output PWM array (-255..255) to /arm_target_angles for serial delivery to the ESP32.
 *
 * Safety:
 *  - on_activate waits for the first /enc_angles message when any motor is closed loop,
 *    so controllers start from the arm's real pose instead of all-zeros. If none arrives,
 *    closed-loop motors stay disabled (0 PWM) until the hardware is restarted.
 *  - A closed-loop motor whose encoder data goes stale (> encoder_timeout_s) gets 0 PWM.
 *  - on_deactivate publishes 0 PWM to every motor.
 *  - While waiting for encoders, 0 PWM is published so the ESP32 (which only streams
 *    telemetry while it receives commands, in older firmware) wakes up.
 *  - Per closed-loop motor FAULTS, each latched at 0 PWM until /arm_hardware/clear_faults:
 *      RUNAWAY - driven one way but moving the other way (reversed encoder sign)
 *      STALL   - commanded to move, >= 50% PWM, but not moving (blocked joint, dead encoder/driver)
 *      JUMP    - encoder angle jumped more than max_jump_deg between two readings
 *      REBOOT  - ESP32 uptime went backwards (encoder zero lost) -> all closed-loop motors
 *    The bevel motors fault together, since the differential wrist needs both.
 *    The gripper is exempt from STALL by default (gripping an object stalls it by design);
 *    its PWM is capped at gripper_pwm_limit instead, which sets the grip force.
 */
class RoverArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverArmHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using SteadyClock = std::chrono::steady_clock;

  // --- HARDWARE DIMENSIONS & INDEXING MAPPING ---
  static constexpr size_t NUM_JOINTS = 7; // ROS joint count (includes mirrored finger)
  static constexpr size_t NUM_MOTORS = 6; // Physical motor channels on ESP32 board

  // ROS Joint Indices (mapped to URDF joint order)
  static constexpr size_t BASE_JOINT_IDX     = 0; // base_link_joint
  static constexpr size_t SHOULDER_JOINT_IDX = 1; // shoulder_link_joint
  static constexpr size_t ELBOW_JOINT_IDX    = 2; // elbow_link_joint
  static constexpr size_t PITCH_JOINT_IDX    = 3; // wrist_link_joint (Wrist Pitch)
  static constexpr size_t ROLL_JOINT_IDX     = 4; // palm_link_joint  (Palm Roll)
  static constexpr size_t GRIPPER_LEFT_IDX   = 5; // left_finger_link_joint
  static constexpr size_t GRIPPER_RIGHT_IDX  = 6; // right_finger_link_joint

  // Expected <ros2_control> joint order. Checked in on_init so a reordered
  // xacro can't silently send base commands to the shoulder motor.
  static constexpr std::array<const char *, NUM_JOINTS> EXPECTED_JOINT_NAMES = {
    "base_link_joint", "shoulder_link_joint", "elbow_link_joint", "wrist_link_joint",
    "palm_link_joint", "left_finger_link_joint", "right_finger_link_joint"};

  // Physical Motor Channel Indices (mapped to ESP32 / AVATAR serial array)
  static constexpr size_t BASE_MOTOR_IDX     = 0; // Base motor
  static constexpr size_t SHOULDER_MOTOR_IDX = 1; // Shoulder motor
  static constexpr size_t ELBOW_MOTOR_IDX    = 2; // Elbow motor
  static constexpr size_t BEVEL1_MOTOR_IDX   = 3; // Bevel 1 motor (Differential Wrist)
  static constexpr size_t BEVEL2_MOTOR_IDX   = 4; // Bevel 2 motor (Differential Wrist)
  static constexpr size_t GRIPPER_MOTOR_IDX  = 5; // Shared Gripper motor

  static constexpr std::array<const char *, NUM_MOTORS> MOTOR_NAMES = {
    "base", "shoulder", "elbow", "bevel1", "bevel2", "gripper"};

  enum class Fault { NONE, RUNAWAY, STALL, JUMP, REBOOT };
  static const char * fault_name(Fault f);

  // Latches a fault on motor m (and its bevel partner) and logs it once.
  // Returns the names of the motors it faulted ("" if none).
  std::string set_fault(size_t m, Fault fault, const std::string & detail, bool log = true);

  // Parses "open_loop_joints" ("none", "all", or comma-separated joint names)
  // into motor_open_loop_. Wrist pitch/roll share both bevel motors, and both
  // fingers share the gripper motor, so naming either joint of a pair puts
  // the shared motor(s) in open loop.
  bool parse_open_loop_joints(const std::string & spec);

  // Motor-space angles/velocities -> joint-space state interfaces.
  void update_joint_states();

  void publish_zero_pwm();

  // Comma-separated names of the motors with motor_open_loop_[m] == open_loop.
  std::string motor_list(bool open_loop) const;

  // ROS Telemetry Callbacks
  void angle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void uptime_callback(const std_msgs::msg::UInt32::SharedPtr msg);

  // --- ROS2_CONTROL BUFFER STORAGE ---
  std::array<double, NUM_JOINTS> hw_velocity_commands_{};
  std::array<double, NUM_JOINTS> hw_positions_{};
  std::array<double, NUM_JOINTS> hw_velocities_{};

  // --- MOTOR-SPACE STATE (encoder or dead-reckoned) ---
  std::array<double, NUM_MOTORS> motor_angle_{};
  std::array<double, NUM_MOTORS> motor_vel_measured_{};
  std::array<double, NUM_MOTORS> motor_vel_setpoint_last_{};
  std::array<bool, NUM_MOTORS> motor_open_loop_{};
  bool encoders_fresh_{false};
  bool closed_loop_disabled_{false}; // set when no /enc_angles arrived during on_activate

  // --- LOGGING STATE (edge-triggered messages) ---
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME}; // valid even while node_ is reset
  bool encoder_dropout_active_{false};
  SteadyClock::time_point dropout_start_{};
  SteadyClock::time_point last_dropout_log_{};
  bool late_encoders_logged_{false};

  // --- FAULT DETECTION (closed-loop motors only) ---
  std::array<Fault, NUM_MOTORS> motor_fault_{};
  std::array<double, NUM_MOTORS> runaway_timer_s_{};
  std::array<double, NUM_MOTORS> stall_timer_s_{};
  std::atomic<bool> clear_faults_requested_{false}; // set by /arm_hardware/clear_faults
  std::atomic<bool> esp_reboot_detected_{false};    // set by /arm_esp_uptime
  SteadyClock::time_point last_fault_reminder_{};

  // --- VELOCITY PI CONTROL STATE (Per Physical Motor) ---
  std::array<double, NUM_MOTORS> motor_vel_error_integral_{};

  // Tunable parameters (overridable via xacro <param> tags)
  double kp_{40.0};
  double ki_{5.0};
  double pwm_limit_{255.0};
  double ff_gain_{100.0};             // PWM per rad/s of motor velocity (open loop, and closed-loop feed-forward)
  double pwm_deadband_{0.0};          // PWM below which the motors don't move (0 = off)
  double gripper_pwm_limit_{180.0};   // gripper PWM cap = grip force when squeezing an object
  bool gripper_stall_fault_{false};   // closing on an object IS a stall, so off by default
  static constexpr double kDeadbandMinCmd = 0.01; // rad/s: only compensate real motion commands
  double encoder_timeout_s_{0.25};    // closed loop: max age of /enc_angles before 0 PWM
  double encoder_wait_s_{10.0};       // on_activate: how long to wait for first /enc_angles
  double velocity_filter_alpha_{0.5}; // 1.0 = no filtering, smaller = smoother
  double runaway_time_s_{0.5};        // 0 disables RUNAWAY detection (keep > braking time)
  double stall_time_s_{1.0};          // 0 disables STALL detection
  double max_jump_deg_{15.0};         // 0 disables JUMP detection
  static constexpr double kRunawayMinVel = 0.05;  // rad/s: "clearly moving the wrong way"
  static constexpr double kRunawayMinPwm = 10.0;  // ignore near-zero PWM noise
  static constexpr double kStallMaxVel = 0.02;    // rad/s: "not moving"
  static constexpr double kStallMinCmd = 0.02;    // rad/s: "a move is being commanded"
  static constexpr double kStallMinPwmFraction = 0.5; // of pwm_limit

  // --- ROS COMMUNICATION & ISOLATED EXECUTOR THREAD ---
  // Thread isolates ROS callbacks from non-deterministic blockages in hardware loops
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr angle_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr uptime_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_faults_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> spinning_{false};

  // Thread-safe raw state storage (written by angle_callback)
  std::mutex state_mutex_;
  std::array<double, NUM_MOTORS> latest_motor_angles_{}; // Radians
  std::array<double, NUM_MOTORS> latest_motor_vels_{};   // Rad/s, filtered
  SteadyClock::time_point last_angle_msg_time_{};
  bool has_angle_data_{false};
  uint32_t last_esp_uptime_ms_{0};
  bool has_esp_uptime_{false};
};

}  // namespace rover_arm_hardware
