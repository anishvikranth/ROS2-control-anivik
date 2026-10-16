#include "rover_arm_hardware/rover_arm_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rover_arm_hardware
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("RoverArmHardware");

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos)
  {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\n\r");
  return s.substr(begin, end - begin + 1);
}

double param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & name,
  double fallback)
{
  const auto it = params.find(name);
  return it == params.end() ? fallback : std::stod(it->second);
}
}  // namespace

hardware_interface::CallbackReturn RoverArmHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != NUM_JOINTS)
  {
    RCLCPP_FATAL(
      kLogger, "Expected %zu joints in <ros2_control>, got %zu", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    if (info_.joints[i].name != EXPECTED_JOINT_NAMES[i])
    {
      RCLCPP_FATAL(
        kLogger, "<ros2_control> joint %zu is '%s', expected '%s'. The joint order must match "
        "RoverArmHardware's motor mapping.", i, info_.joints[i].name.c_str(),
        EXPECTED_JOINT_NAMES[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_velocity_commands_.fill(0.0);
  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  motor_angle_.fill(0.0);
  motor_vel_measured_.fill(0.0);
  motor_vel_setpoint_last_.fill(0.0);
  motor_vel_error_integral_.fill(0.0);
  latest_motor_angles_.fill(0.0);
  latest_motor_vels_.fill(0.0);
  motor_open_loop_.fill(false);

  // Read tuning parameters from URDF/Xacro if provided
  try
  {
    const auto & hw_params = info_.hardware_parameters;
    kp_ = param_or(hw_params, "kp", kp_);
    ki_ = param_or(hw_params, "ki", ki_);
    pwm_limit_ = param_or(hw_params, "pwm_limit", pwm_limit_);
    ff_gain_ = param_or(hw_params, "ff_gain", ff_gain_);
    encoder_timeout_s_ = param_or(hw_params, "encoder_timeout_s", encoder_timeout_s_);
    encoder_wait_s_ = param_or(hw_params, "encoder_wait_s", encoder_wait_s_);
    velocity_filter_alpha_ =
      std::clamp(param_or(hw_params, "velocity_filter_alpha", velocity_filter_alpha_), 0.01, 1.0);
    runaway_time_s_ = param_or(hw_params, "runaway_time_s", runaway_time_s_);
    stall_time_s_ = param_or(hw_params, "stall_time_s", stall_time_s_);
    max_jump_deg_ = param_or(hw_params, "max_jump_deg", max_jump_deg_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(kLogger, "Invalid numeric hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    kLogger, "Parameters: kp=%.2f ki=%.2f pwm_limit=%.0f ff_gain=%.1f encoder_timeout_s=%.2f "
    "encoder_wait_s=%.1f velocity_filter_alpha=%.2f runaway_time_s=%.2f stall_time_s=%.2f "
    "max_jump_deg=%.1f (0 = check disabled)", kp_, ki_, pwm_limit_, ff_gain_,
    encoder_timeout_s_, encoder_wait_s_, velocity_filter_alpha_, runaway_time_s_, stall_time_s_,
    max_jump_deg_);

  const auto spec_it = info_.hardware_parameters.find("open_loop_joints");
  if (!parse_open_loop_joints(spec_it == info_.hardware_parameters.end() ? "none" : spec_it->second))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RoverArmHardware::parse_open_loop_joints(const std::string & spec)
{
  const std::string trimmed = trim(spec);
  if (trimmed.empty() || trimmed == "none")
  {
    RCLCPP_INFO(kLogger, "All motors CLOSED LOOP (encoders required).");
    return true;
  }
  if (trimmed == "all")
  {
    motor_open_loop_.fill(true);
  }
  else
  {
    std::stringstream ss(trimmed);
    std::string name;
    while (std::getline(ss, name, ','))
    {
      name = trim(name);
      if (name.empty())
      {
        continue;
      }
      const auto it = std::find(EXPECTED_JOINT_NAMES.begin(), EXPECTED_JOINT_NAMES.end(), name);
      if (it == EXPECTED_JOINT_NAMES.end())
      {
        RCLCPP_FATAL(kLogger, "open_loop_joints: unknown joint '%s'", name.c_str());
        return false;
      }
      switch (static_cast<size_t>(it - EXPECTED_JOINT_NAMES.begin()))
      {
        case BASE_JOINT_IDX:     motor_open_loop_[BASE_MOTOR_IDX] = true; break;
        case SHOULDER_JOINT_IDX: motor_open_loop_[SHOULDER_MOTOR_IDX] = true; break;
        case ELBOW_JOINT_IDX:    motor_open_loop_[ELBOW_MOTOR_IDX] = true; break;
        case PITCH_JOINT_IDX:
        case ROLL_JOINT_IDX:
          motor_open_loop_[BEVEL1_MOTOR_IDX] = true;
          motor_open_loop_[BEVEL2_MOTOR_IDX] = true;
          break;
        case GRIPPER_LEFT_IDX:
        case GRIPPER_RIGHT_IDX:  motor_open_loop_[GRIPPER_MOTOR_IDX] = true; break;
      }
    }
  }

  RCLCPP_WARN(
    kLogger, "OPEN LOOP motors: [%s] (pwm = %.1f * velocity, positions are dead-reckoned "
    "estimates -- do not trust them for MoveIt). CLOSED LOOP motors: [%s].",
    motor_list(true).c_str(), ff_gain_, motor_list(false).c_str());
  return true;
}

std::string RoverArmHardware::motor_list(bool open_loop) const
{
  std::string list;
  for (size_t m = 0; m < NUM_MOTORS; ++m)
  {
    if (motor_open_loop_[m] == open_loop)
    {
      list += std::string(list.empty() ? "" : ", ") + MOTOR_NAMES[m];
    }
  }
  return list.empty() ? "none" : list;
}

const char * RoverArmHardware::fault_name(Fault f)
{
  switch (f)
  {
    case Fault::RUNAWAY: return "RUNAWAY";
    case Fault::STALL:   return "STALL";
    case Fault::JUMP:    return "JUMP";
    case Fault::REBOOT:  return "REBOOT";
    default:             return "NONE";
  }
}

std::string RoverArmHardware::set_fault(
  size_t m, Fault fault, const std::string & detail, bool log)
{
  if (motor_open_loop_[m] || motor_fault_[m] != Fault::NONE)
  {
    return "";
  }
  std::string motors = MOTOR_NAMES[m];
  motor_fault_[m] = fault;
  motor_vel_error_integral_[m] = 0.0;

  // The differential wrist needs both bevel motors: stopping only one would
  // turn a pitch command into an uncontrolled roll (and vice versa).
  const size_t partner =
    m == BEVEL1_MOTOR_IDX ? BEVEL2_MOTOR_IDX : (m == BEVEL2_MOTOR_IDX ? BEVEL1_MOTOR_IDX : m);
  if (partner != m && motor_fault_[partner] == Fault::NONE)
  {
    motor_fault_[partner] = fault;
    motor_vel_error_integral_[partner] = 0.0;
    motors += std::string(" + ") + MOTOR_NAMES[partner] + " (wrist pair)";
  }
  if (!log)
  {
    return motors;
  }

  const char * hint = "";
  switch (fault)
  {
    case Fault::RUNAWAY:
      hint = "The encoder direction is probably reversed relative to the motor (closed loop "
             "became positive feedback). Fix enc_dir[] or dir_dir[] in the ESP32 firmware.";
      break;
    case Fault::STALL:
      hint = "Joint mechanically blocked, encoder not counting, or motor/driver failure.";
      break;
    case Fault::JUMP:
      hint = "Encoder glitch, noise on the encoder wires, or an ESP32 reset.";
      break;
    case Fault::REBOOT:
      hint = "Encoder counts restarted from 0, so positions are now wrong. Put the arm in its "
             "zero pose and restart the arm launch.";
      break;
    default:
      break;
  }
  RCLCPP_ERROR(
    kLogger, "%s FAULT on [%s]: %s -- motor(s) DISABLED, PWM = 0. %s After fixing: switch to "
    "JOYSTICK_ONLY (button 2) so nothing is commanded, then run: "
    "ros2 topic pub --once /arm_hardware/clear_faults std_msgs/msg/Empty",
    fault_name(fault), motors.c_str(), detail.c_str(), hint);
  last_fault_reminder_ = SteadyClock::now();
  return motors;
}

std::vector<hardware_interface::StateInterface> RoverArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RoverArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("rover_arm_hardware_bridge");

  target_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/arm_target_angles", rclcpp::QoS(10));

  angle_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/enc_angles", rclcpp::QoS(10),
    std::bind(&RoverArmHardware::angle_callback, this, std::placeholders::_1));

  uptime_sub_ = node_->create_subscription<std_msgs::msg::UInt32>(
    "/arm_esp_uptime", rclcpp::QoS(10),
    std::bind(&RoverArmHardware::uptime_callback, this, std::placeholders::_1));

  clear_faults_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    "/arm_hardware/clear_faults", rclcpp::QoS(10),
    [this](std_msgs::msg::Empty::SharedPtr) { clear_faults_requested_ = true; });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  // Spin until on_deactivate clears spinning_. (executor_->cancel() alone does
  // not stop a spin_some() loop, which used to make on_deactivate hang in join().)
  spinning_ = true;
  spin_thread_ = std::make_unique<std::thread>([this]() {
    while (spinning_ && rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  hw_velocity_commands_.fill(0.0);
  motor_vel_setpoint_last_.fill(0.0);
  motor_vel_error_integral_.fill(0.0);
  closed_loop_disabled_ = false;
  encoder_dropout_active_ = false;
  late_encoders_logged_ = false;
  motor_fault_.fill(Fault::NONE);
  runaway_timer_s_.fill(0.0);
  stall_timer_s_.fill(0.0);
  clear_faults_requested_ = false;
  esp_reboot_detected_ = false;

  // Controllers capture the current state as their hold position when they
  // activate, so the state must be the arm's real pose -- not zeros -- or the
  // arm_controller would drive every joint to 0 as soon as encoders appear.
  const bool any_closed_loop =
    std::any_of(motor_open_loop_.begin(), motor_open_loop_.end(), [](bool ol) { return !ol; });
  if (any_closed_loop)
  {
    RCLCPP_INFO(kLogger, "Waiting up to %.1f s for /enc_angles...", encoder_wait_s_);
    const auto deadline = SteadyClock::now() + std::chrono::duration<double>(encoder_wait_s_);
    bool got_data = false;
    while (SteadyClock::now() < deadline && rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        got_data = has_angle_data_;
      }
      if (got_data)
      {
        break;
      }
      // Keep the ESP32 link alive with harmless 0 PWM. Firmware that only
      // streams encoders while it receives commands would otherwise never
      // send any, and this wait would always time out.
      publish_zero_pwm();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // Returning ERROR here would make the Jazzy controller_manager abort the
    // whole ros2_control_node at startup, so instead stay active with the
    // closed-loop motors disabled. They are NOT re-enabled if encoders show up
    // later: the controllers would already be holding the fake 0 pose and the
    // arm would jump there.
    closed_loop_disabled_ = !got_data;
    if (closed_loop_disabled_)
    {
      RCLCPP_ERROR(
        kLogger, "No /enc_angles received in %.1f s -- closed-loop motors DISABLED (0 PWM) "
        "until the hardware is restarted. Check that arm.py is running and the encoders are "
        "connected. To drive WITHOUT encoders, relaunch with open_loop_joints:=all (or list "
        "the joints whose encoders are missing).", encoder_wait_s_);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t m = 0; m < NUM_MOTORS; ++m)
    {
      motor_angle_[m] = (motor_open_loop_[m] || !got_data) ? 0.0 : latest_motor_angles_[m];
      motor_vel_measured_[m] = 0.0;
    }
    if (got_data)
    {
      constexpr double RAD_TO_DEG = 180.0 / M_PI;
      RCLCPP_INFO(
        kLogger, "Encoders detected. Start pose (motor deg): base=%.1f shoulder=%.1f elbow=%.1f "
        "bevel1=%.1f bevel2=%.1f gripper=%.1f", latest_motor_angles_[0] * RAD_TO_DEG,
        latest_motor_angles_[1] * RAD_TO_DEG, latest_motor_angles_[2] * RAD_TO_DEG,
        latest_motor_angles_[3] * RAD_TO_DEG, latest_motor_angles_[4] * RAD_TO_DEG,
        latest_motor_angles_[5] * RAD_TO_DEG);
    }
  }
  else
  {
    motor_angle_.fill(0.0);
    motor_vel_measured_.fill(0.0);
    RCLCPP_WARN(
      kLogger, "All motors open loop: dead-reckoned positions start at 0. Start with the "
      "arm in its zero pose so the estimates mean something.");
  }
  update_joint_states();

  RCLCPP_INFO(kLogger, "Activated: publishing to /arm_target_angles, reading /enc_angles");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publish_zero_pwm();
  RCLCPP_INFO(kLogger, "Deactivating: PWM = 0 sent to all motors.");

  spinning_ = false;
  if (spin_thread_ && spin_thread_->joinable())
  {
    spin_thread_->join();
  }
  spin_thread_.reset();
  angle_sub_.reset();
  uptime_sub_.reset();
  clear_faults_sub_.reset();
  target_pub_.reset();
  executor_.reset();
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RoverArmHardware::publish_zero_pwm()
{
  if (!target_pub_)
  {
    return;
  }
  std_msgs::msg::Float32MultiArray msg;
  msg.data.assign(NUM_MOTORS, 0.0f);
  target_pub_->publish(msg);
}

void RoverArmHardware::angle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < NUM_MOTORS)
  {
    RCLCPP_WARN_THROTTLE(
      kLogger, throttle_clock_, 5000,
      "/enc_angles has %zu values, expected at least %zu", msg->data.size(), NUM_MOTORS);
    return;
  }

  for (size_t i = 0; i < NUM_MOTORS; ++i)
  {
    if (!std::isfinite(msg->data[i]))
    {
      RCLCPP_WARN_THROTTLE(
        kLogger, throttle_clock_, 2000,
        "/enc_angles[%zu] (%s) is not a finite number -- message ignored. Check that encoder's "
        "wiring/firmware.", i, MOTOR_NAMES[i]);
      return;
    }
  }

  // Convert raw incoming encoder degrees from ESP32 to radians
  constexpr double DEG_TO_RAD = M_PI / 180.0;
  const auto now = SteadyClock::now();

  std::lock_guard<std::mutex> lock(state_mutex_);
  // Velocity is differentiated between consecutive *messages*, not between
  // read() calls: read() runs at 100 Hz, faster than /enc_angles typically
  // arrives, which would otherwise give alternating zeros and spikes.
  const double dt =
    has_angle_data_ ? std::chrono::duration<double>(now - last_angle_msg_time_).count() : 0.0;
  for (size_t i = 0; i < NUM_MOTORS; ++i)
  {
    const double angle = static_cast<double>(msg->data[i]) * DEG_TO_RAD;
    if (dt > 1e-4)
    {
      const double raw_vel = (angle - latest_motor_angles_[i]) / dt;
      latest_motor_vels_[i] =
        velocity_filter_alpha_ * raw_vel + (1.0 - velocity_filter_alpha_) * latest_motor_vels_[i];
    }
    latest_motor_angles_[i] = angle;
  }
  last_angle_msg_time_ = now;
  has_angle_data_ = true;
}

void RoverArmHardware::uptime_callback(const std_msgs::msg::UInt32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (has_esp_uptime_ && msg->data < last_esp_uptime_ms_)
  {
    esp_reboot_detected_ = true; // handled (and logged) in read()
  }
  last_esp_uptime_ms_ = msg->data;
  has_esp_uptime_ = true;
}

hardware_interface::return_type RoverArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!node_)
  {
    return hardware_interface::return_type::OK; // inactive: nothing to read
  }

  std::array<double, NUM_MOTORS> enc_angles{};
  std::array<double, NUM_MOTORS> enc_vels{};
  bool have_data = false;
  SteadyClock::time_point last_msg_time;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enc_angles = latest_motor_angles_;
    enc_vels = latest_motor_vels_;
    have_data = has_angle_data_;
    last_msg_time = last_angle_msg_time_;
  }

  encoders_fresh_ =
    !closed_loop_disabled_ && have_data &&
    std::chrono::duration<double>(SteadyClock::now() - last_msg_time).count() < encoder_timeout_s_;

  if (clear_faults_requested_.exchange(false))
  {
    std::string cleared;
    bool cleared_reboot = false;
    for (size_t m = 0; m < NUM_MOTORS; ++m)
    {
      if (motor_fault_[m] != Fault::NONE)
      {
        cleared_reboot = cleared_reboot || motor_fault_[m] == Fault::REBOOT;
        cleared += std::string(cleared.empty() ? "" : ", ") + MOTOR_NAMES[m] + " (" +
          fault_name(motor_fault_[m]) + ")";
        motor_fault_[m] = Fault::NONE;
      }
      runaway_timer_s_[m] = 0.0;
      stall_timer_s_[m] = 0.0;
      motor_vel_error_integral_[m] = 0.0;
    }
    if (cleared.empty())
    {
      RCLCPP_INFO(kLogger, "clear_faults received, but no motor was faulted.");
    }
    else
    {
      RCLCPP_WARN(
        kLogger, "Faults CLEARED on [%s]. Those motors follow the active controller's command "
        "again from now on.", cleared.c_str());
      if (cleared_reboot)
      {
        RCLCPP_WARN(
          kLogger, "A REBOOT fault was cleared without restarting: joint positions are now "
          "measured from wherever the arm was when the ESP32 reset, NOT from the true zero. "
          "Joint limits, RViz and MoveIt are offset until you re-zero and restart.");
      }
    }
  }

  if (esp_reboot_detected_.exchange(false))
  {
    std::string faulted;
    for (size_t m = 0; m < NUM_MOTORS; ++m)
    {
      const std::string names = set_fault(m, Fault::REBOOT, "", false);
      if (!names.empty())
      {
        faulted += (faulted.empty() ? "" : ", ") + names;
      }
    }
    RCLCPP_ERROR(
      kLogger, "ESP32 REBOOTED mid-run (its uptime went backwards): encoder counts restarted "
      "from 0, so positions are now wrong. REBOOT FAULT -- closed-loop motors DISABLED, PWM = 0: "
      "[%s]. Put the arm in its zero pose and restart the arm launch.",
      faulted.empty() ? "none (all already faulted or open loop)" : faulted.c_str());
    last_fault_reminder_ = SteadyClock::now();
  }

  const double dt = period.seconds();
  const double max_jump_rad = max_jump_deg_ * M_PI / 180.0;
  bool any_closed_loop_stale = false;
  for (size_t m = 0; m < NUM_MOTORS; ++m)
  {
    if (motor_open_loop_[m])
    {
      // Dead reckoning: assume the motor did what it was told last cycle.
      motor_vel_measured_[m] = motor_vel_setpoint_last_[m];
      motor_angle_[m] += motor_vel_setpoint_last_[m] * dt;
    }
    else if (encoders_fresh_)
    {
      // A real joint can't move max_jump_deg in one 10 ms reading.
      const double jump = std::abs(enc_angles[m] - motor_angle_[m]);
      if (max_jump_rad > 0.0 && jump > max_jump_rad)
      {
        set_fault(
          m, Fault::JUMP, "encoder jumped " + std::to_string(jump * 180.0 / M_PI).substr(0, 6) +
          " deg between two readings");
      }
      motor_angle_[m] = enc_angles[m];
      motor_vel_measured_[m] = enc_vels[m];
    }
    else
    {
      // Hold the last known angle; write() outputs 0 PWM for this motor.
      motor_vel_measured_[m] = 0.0;
      any_closed_loop_stale = true;
    }
  }

  if (any_closed_loop_stale && closed_loop_disabled_)
  {
    if (have_data && !late_encoders_logged_)
    {
      RCLCPP_WARN(
        kLogger, "Encoder data is arriving now, but closed-loop motors [%s] stay DISABLED "
        "(PWM = 0) because the hardware started without encoders. Restart to use them.",
        motor_list(false).c_str());
      late_encoders_logged_ = true;
    }
    RCLCPP_ERROR_THROTTLE(
      kLogger, throttle_clock_, 5000,
      "Closed-loop motors [%s] disabled, PWM = 0 (no encoders at startup). Restart, or "
      "relaunch with open_loop_joints:=all.", motor_list(false).c_str());
  }
  else if (any_closed_loop_stale)
  {
    const auto now = SteadyClock::now();
    if (!encoder_dropout_active_)
    {
      // Logged once, the moment the dropout is detected.
      encoder_dropout_active_ = true;
      dropout_start_ = now;
      last_dropout_log_ = now;
      RCLCPP_ERROR(
        kLogger, "ENCODERS UNPLUGGED / no /enc_angles for > %.2f s -- setting PWM = 0 on "
        "closed-loop motors [%s].", encoder_timeout_s_, motor_list(false).c_str());
    }
    else if (now - last_dropout_log_ >= std::chrono::seconds(2))
    {
      // Reminder every 2 s (timed by hand: a throttle macro would also fire
      // right after the one-shot message above).
      last_dropout_log_ = now;
      RCLCPP_ERROR(
        kLogger,
        "Encoders still unplugged (%.1f s) -- PWM = 0 on closed-loop motors [%s]. When they "
        "reconnect, the arm resumes toward its last target.",
        std::chrono::duration<double>(now - dropout_start_).count(), motor_list(false).c_str());
    }
  }
  else if (encoder_dropout_active_)
  {
    encoder_dropout_active_ = false;
    RCLCPP_WARN(
      kLogger, "ENCODERS RECONNECTED after %.1f s -- closed-loop control resumed on [%s]. "
      "The arm will now move toward its last commanded target.",
      std::chrono::duration<double>(SteadyClock::now() - dropout_start_).count(),
      motor_list(false).c_str());
  }

  if (std::any_of(motor_fault_.begin(), motor_fault_.end(), [](Fault f) { return f != Fault::NONE; }) &&
    SteadyClock::now() - last_fault_reminder_ >= std::chrono::seconds(5))
  {
    last_fault_reminder_ = SteadyClock::now();
    std::string list;
    for (size_t m = 0; m < NUM_MOTORS; ++m)
    {
      if (motor_fault_[m] != Fault::NONE)
      {
        list += std::string(list.empty() ? "" : ", ") + MOTOR_NAMES[m] + " (" +
          fault_name(motor_fault_[m]) + ")";
      }
    }
    RCLCPP_ERROR(
      kLogger, "Motors still FAULTED, PWM = 0: [%s]. Clear with: ros2 topic pub --once "
      "/arm_hardware/clear_faults std_msgs/msg/Empty", list.c_str());
  }

  update_joint_states();
  return hardware_interface::return_type::OK;
}

void RoverArmHardware::update_joint_states()
{
  // --- 1:1 Joint Mappings ---
  hw_positions_[BASE_JOINT_IDX]     = motor_angle_[BASE_MOTOR_IDX];
  hw_velocities_[BASE_JOINT_IDX]    = motor_vel_measured_[BASE_MOTOR_IDX];

  hw_positions_[SHOULDER_JOINT_IDX] = motor_angle_[SHOULDER_MOTOR_IDX];
  hw_velocities_[SHOULDER_JOINT_IDX]= motor_vel_measured_[SHOULDER_MOTOR_IDX];

  hw_positions_[ELBOW_JOINT_IDX]    = motor_angle_[ELBOW_MOTOR_IDX];
  hw_velocities_[ELBOW_JOINT_IDX]   = motor_vel_measured_[ELBOW_MOTOR_IDX];

  // --- Forward Differential Wrist Kinematics (Bevel Motors -> Pitch & Roll) ---
  hw_positions_[PITCH_JOINT_IDX]  = (motor_angle_[BEVEL1_MOTOR_IDX] + motor_angle_[BEVEL2_MOTOR_IDX]) / 2.0;
  hw_velocities_[PITCH_JOINT_IDX] = (motor_vel_measured_[BEVEL1_MOTOR_IDX] + motor_vel_measured_[BEVEL2_MOTOR_IDX]) / 2.0;

  hw_positions_[ROLL_JOINT_IDX]   = (motor_angle_[BEVEL1_MOTOR_IDX] - motor_angle_[BEVEL2_MOTOR_IDX]) / 2.0;
  hw_velocities_[ROLL_JOINT_IDX]  = (motor_vel_measured_[BEVEL1_MOTOR_IDX] - motor_vel_measured_[BEVEL2_MOTOR_IDX]) / 2.0;

  // --- Mirrored Finger Joint Mapping ---
  hw_positions_[GRIPPER_LEFT_IDX]   = motor_angle_[GRIPPER_MOTOR_IDX];
  hw_velocities_[GRIPPER_LEFT_IDX]  = motor_vel_measured_[GRIPPER_MOTOR_IDX];

  hw_positions_[GRIPPER_RIGHT_IDX]  = -motor_angle_[GRIPPER_MOTOR_IDX];
  hw_velocities_[GRIPPER_RIGHT_IDX] = -motor_vel_measured_[GRIPPER_MOTOR_IDX];
}

hardware_interface::return_type RoverArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!node_)
  {
    return hardware_interface::return_type::OK; // inactive: never drive motors
  }

  std::array<double, NUM_MOTORS> motor_vel_setpoint{};

  // --- 1:1 Joint Commands ---
  motor_vel_setpoint[BASE_MOTOR_IDX]     = hw_velocity_commands_[BASE_JOINT_IDX];
  motor_vel_setpoint[SHOULDER_MOTOR_IDX] = hw_velocity_commands_[SHOULDER_JOINT_IDX];
  motor_vel_setpoint[ELBOW_MOTOR_IDX]    = hw_velocity_commands_[ELBOW_JOINT_IDX];

  // --- Inverse Differential Wrist Kinematics (Pitch & Roll -> Bevel Motors) ---
  const double pitch_cmd = hw_velocity_commands_[PITCH_JOINT_IDX];
  const double roll_cmd  = hw_velocity_commands_[ROLL_JOINT_IDX];

  motor_vel_setpoint[BEVEL1_MOTOR_IDX] = pitch_cmd + roll_cmd;
  motor_vel_setpoint[BEVEL2_MOTOR_IDX] = pitch_cmd - roll_cmd;

  // --- Gripper Combination ---
  motor_vel_setpoint[GRIPPER_MOTOR_IDX] =
    (hw_velocity_commands_[GRIPPER_LEFT_IDX] - hw_velocity_commands_[GRIPPER_RIGHT_IDX]) / 2.0;

  // NaN commands (e.g. a controller that never wrote) must never reach the motors.
  for (size_t m = 0; m < NUM_MOTORS; ++m)
  {
    if (!std::isfinite(motor_vel_setpoint[m]))
    {
      RCLCPP_WARN_THROTTLE(
        kLogger, throttle_clock_, 5000,
        "Non-finite velocity command for %s motor (no active controller writing it?) -- "
        "using 0.", MOTOR_NAMES[m]);
      motor_vel_setpoint[m] = 0.0;
    }
  }

  // --- Per-motor: open-loop feed-forward, closed-loop PI, or stop ---
  const double dt = std::max(period.seconds(), 1e-6);
  const double integral_limit = pwm_limit_ / std::max(ki_, 1e-6);

  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(NUM_MOTORS);

  for (size_t m = 0; m < NUM_MOTORS; ++m)
  {
    double pwm = 0.0;
    if (motor_open_loop_[m])
    {
      pwm = ff_gain_ * motor_vel_setpoint[m];
      motor_vel_error_integral_[m] = 0.0;
    }
    else if (motor_fault_[m] != Fault::NONE)
    {
      motor_vel_error_integral_[m] = 0.0; // latched fault: stay at 0 PWM
    }
    else if (encoders_fresh_)
    {
      const double error = motor_vel_setpoint[m] - motor_vel_measured_[m];
      motor_vel_error_integral_[m] =
        std::clamp(motor_vel_error_integral_[m] + error * dt, -integral_limit, integral_limit);
      pwm = kp_ * error + ki_ * motor_vel_error_integral_[m];

      // Fault detection.
      // RUNAWAY: the motor moves opposite to the direction it is driven. With a
      // reversed encoder the PI never saturates quickly (the PWM creeps up while
      // the joint drives the wrong way), so this must not wait for saturation.
      // Braking after a reversal also looks like this briefly -- runaway_time_s
      // must stay longer than that.
      // STALL: a move is commanded and the PI pushes hard, but nothing moves.
      const double vel = motor_vel_measured_[m];
      const bool wrong_way =
        std::abs(pwm) > kRunawayMinPwm && pwm * vel < 0.0 && std::abs(vel) > kRunawayMinVel;
      const bool stalled = std::abs(motor_vel_setpoint[m]) > kStallMinCmd &&
        std::abs(pwm) >= kStallMinPwmFraction * pwm_limit_ && std::abs(vel) < kStallMaxVel;
      runaway_timer_s_[m] = wrong_way ? runaway_timer_s_[m] + dt : 0.0;
      stall_timer_s_[m] = stalled ? stall_timer_s_[m] + dt : 0.0;

      char detail[160];
      if (runaway_time_s_ > 0.0 && runaway_timer_s_[m] >= runaway_time_s_)
      {
        std::snprintf(
          detail, sizeof(detail), "driven at PWM %.0f but moving the WRONG way for %.2f s (cmd "
          "%.2f rad/s, measured %.2f rad/s)", pwm, runaway_timer_s_[m], motor_vel_setpoint[m], vel);
        set_fault(m, Fault::RUNAWAY, detail);
        pwm = 0.0;
      }
      else if (stall_time_s_ > 0.0 && stall_timer_s_[m] >= stall_time_s_)
      {
        std::snprintf(
          detail, sizeof(detail), "PWM %.0f for %.2f s but not moving (cmd %.2f rad/s, measured "
          "%.3f rad/s)", pwm, stall_timer_s_[m], motor_vel_setpoint[m], vel);
        set_fault(m, Fault::STALL, detail);
        pwm = 0.0;
      }
    }
    else
    {
      // No feedback: stop the motor instead of winding up to full PWM.
      motor_vel_error_integral_[m] = 0.0;
    }
    if (std::abs(pwm) > pwm_limit_)
    {
      RCLCPP_WARN_THROTTLE(
        kLogger, throttle_clock_, 5000,
        "%s motor PWM saturated at %.0f (wanted %.0f, %s). It cannot reach the commanded "
        "velocity %.2f rad/s -- lower the speed or retune.", MOTOR_NAMES[m], pwm_limit_,
        pwm, motor_open_loop_[m] ? "open loop" : "closed loop", motor_vel_setpoint[m]);
    }
    pwm = std::clamp(pwm, -pwm_limit_, pwm_limit_);
    msg.data[m] = static_cast<float>(pwm);

    // Dead reckoning in read() integrates what was actually requested of
    // the motor. Saturated PWM means the real motion lags this estimate.
    motor_vel_setpoint_last_[m] = motor_vel_setpoint[m];
  }

  // A fault raised late in the loop can also latch an earlier motor (bevel pair).
  for (size_t m = 0; m < NUM_MOTORS; ++m)
  {
    if (!motor_open_loop_[m] && motor_fault_[m] != Fault::NONE)
    {
      msg.data[m] = 0.0f;
    }
  }

  // Enable with: --ros-args --log-level RoverArmHardware:=debug
  RCLCPP_DEBUG_THROTTLE(
    kLogger, throttle_clock_, 1000, "PWM out: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f]",
    msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);

  if (target_pub_)
  {
    target_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace rover_arm_hardware

PLUGINLIB_EXPORT_CLASS(rover_arm_hardware::RoverArmHardware, hardware_interface::SystemInterface)
