# Rover Arm – ROS 2 Control, MoveIt & Simulation

ROS 2 (Jazzy) workspace for Team Anveshak's rover arm: the robot description, Gazebo
simulation, MoveIt configuration, and a `ros2_control` hardware plugin that drives the
real arm through the AVATAR serial link to the ESP32.

The same controllers and MoveIt setup run in simulation and on the real arm; only the
hardware layer underneath changes.

```
 Joystick (/joy_arm) ──► real_arm_teleop_node ──┐
                                                ├──► arm_controller (JTC) ─────────┐
 MoveIt (move_group) ───────────────────────────┘    or joystick_only_controller ─┤
                                                                                  ▼
                              Gazebo (gz_ros2_control)   or   RoverArmHardware plugin
                                                                  │  /arm_target_angles (PWM)
                                                                  ▼        ▲ /enc_angles (deg)
                                                               arm.py  ◄──► ESP32 (arm.c)
```

## Repository layout

| Path | What it is |
|---|---|
| `src/rover_arm_description` | Arm URDFs (`rover_arm_raw/sim/real.xacro`), meshes, controller configs, launch files, teleop and MoveIt test nodes |
| `src/rover_arm_hardware` | `ros2_control` SystemInterface plugin for the real arm (`RoverArmHardware`) |
| `src/rover_arm_moveit_config` | MoveIt config (SRDF, kinematics, joint limits, move_group/RViz launches) |
| `src/rover_description` | Full rover (rocker-bogie) description and Gazebo world with ArUco markers |
| `src/jade_description` | Rover + arm description (work in progress) |
| `src/rover_aruco`, `src/rover_aruco_interfaces` | ArUco detection, marker tracking and navigation-goal generation |
| `avatar_files/` | Copies of the AVATAR files used with the real arm: `arm.c` (ESP32 firmware with encoders) and `arm.py` (Orin-side serial bridge). See [AVATAR files](#avatar-files). |

## Build

Requires ROS 2 Jazzy with `ros2_control`, `ros2_controllers`, MoveIt 2, `ros_gz` and `joy`.

```bash
cd <this repo>
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Simulation

| Command | What it starts |
|---|---|
| `ros2 launch rover_arm_description gazebo.launch.py` | Gazebo + arm, controllers, RViz, `joy_node` and the sim joystick node, a target cube, camera bridges (gripper camera, ZED RGB/depth/IMU) |
| `ros2 launch rover_arm_description arm.launch.py` | The above plus MoveIt (`move_group` + MoveIt RViz) |
| `ros2 launch rover_description gz.launch.py` | Full rover in Gazebo |

The simulated arm uses **position** command interfaces; the real arm uses **velocity**.
Joystick-only mode (below) therefore exists on the real arm only, for now.

## Real arm

### Bring-up

1. Flash `avatar_files/arm.c` to the ESP32 (needs the AVATAR Arduino library).
2. On the Orin, clone the AVATAR repo to `~/AVATAR`, `pip install` its `python/` package, and
   make sure `~/AVATAR/orin_side/arm.py` is the version in `avatar_files/arm.py`.
3. Publish the joystick on `/joy_arm` (e.g. `joy_node` remapped from `/joy`).
4. Launch:

```bash
# Encoders working (normal, closed loop)
ros2 launch rover_arm_description arm_real.launch.py

# No encoders: every motor open loop, starts in joystick-only mode
ros2 launch rover_arm_description arm_real.launch.py open_loop_joints:=all

# Only some encoders dead (e.g. the wrist)
ros2 launch rover_arm_description arm_real.launch.py open_loop_joints:=wrist_link_joint,palm_link_joint
```

| Launch argument | Default | Meaning |
|---|---|---|
| `open_loop_joints` | `none` | Joints driven without encoder feedback: `none`, `all`, or comma-separated joint names. Anything other than `none` starts in joystick-only mode. |
| `avatar_repo_path` | `~/AVATAR` | Where `orin_side/arm.py` is run from |
| `time_from_start_s` | `0.15` | Trajectory point time used by the teleop node |
| `cooldown_s` | `0.3` | Debounce for the mode buttons |

`arm_real.launch.py` starts `arm.py`, `robot_state_publisher`, `ros2_control_node` with
`RoverArmHardware`, the controllers, the teleop node, `move_group` and MoveIt RViz.

### Control modes

| Mode | Controller | How the joystick drives the arm | Needs encoders |
|---|---|---|---|
| `MANUAL` | `arm_controller` (JointTrajectoryController) | Moves target joint **positions** | Yes |
| `JOYSTICK_ONLY` | `joystick_only_controller` (JointGroupVelocityController) | Sends joint **velocities** straight to the motors, no position loop | No |
| `AUTONOMOUS` | `arm_controller`, driven by MoveIt | Joystick ignored (except the mode button) | Yes |

The current mode is published on `/arm_control_mode`. Only one of the two controllers is
active at a time; the teleop node switches them through the controller manager.

### Joystick mapping (`/joy_arm`)

| Input | Action |
|---|---|
| `axes[0]` / `axes[1]` / `axes[4]` | Base / shoulder / elbow |
| `axes[7]` / `axes[6]` | Wrist pitch / palm roll |
| `axes[3]` | Gripper open / close |
| `button[0]` | Toggle MANUAL ↔ AUTONOMOUS |
| `button[1]` | Toggle joint space ↔ position space (published on `/input_space`, not acted on yet) |
| `button[2]` | Toggle MANUAL ↔ JOYSTICK_ONLY (param `joystick_only_button_idx`) |
| `button[6]` / `button[7]` | Speed tier down / up (position step 0.005/0.01/0.02 rad, velocity 0.1/0.25/0.5 rad/s) |

Mode buttons toggle once per press; holding a button does not repeat.

### Safety behaviour

| Situation | What happens |
|---|---|
| No `/enc_angles` at startup (closed-loop motors) | The plugin waits `encoder_wait_s`, sending 0 PWM to wake the ESP32; if nothing arrives, closed-loop motors stay disabled (0 PWM) until restart |
| Encoders stop mid-run | Closed-loop motors get 0 PWM after `encoder_timeout_s`; resumes when data returns |
| Joystick link lost (> 0.5 s) | JOYSTICK_ONLY: velocities forced to 0. MANUAL: arm holds the last target |
| No command reaches the ESP32 for 1 s | ESP32 firmware stops all motors (LED red) |
| Hardware deactivated | 0 PWM sent to all motors |

**Faults** (closed-loop motors). A fault latches the motor at 0 PWM; the two bevel motors
fault together because the differential wrist needs both.

| Fault | Trigger | Usual cause |
|---|---|---|
| `RUNAWAY` | Driven one way but moving the other way for `runaway_time_s` | Encoder direction reversed (fix `enc_dir[]` / `dir_dir[]` in `arm.c`) |
| `STALL` | Move commanded, ≥ 50 % PWM, not moving for `stall_time_s` | Joint blocked, encoder not counting, motor/driver failure |
| `JUMP` | Encoder jumps more than `max_jump_deg` between readings | Encoder noise/glitch, ESP32 reset |
| `REBOOT` | ESP32 uptime went backwards (all closed-loop motors) | ESP32 reset: encoder counts restarted from 0, positions are wrong |

To clear faults after fixing the cause: switch to JOYSTICK_ONLY (button 2) so nothing is
commanded, then

```bash
ros2 topic pub --once /arm_hardware/clear_faults std_msgs/msg/Empty
```

After a `REBOOT` fault, put the arm in its zero pose and restart instead. The encoders are
incremental, so **the arm must be in its URDF zero pose when the ESP32 powers up**.

### Hardware parameters (`rover_arm_real.xacro`)

| Parameter | Default | Meaning |
|---|---|---|
| `kp`, `ki` | 40, 5 | Velocity PI gains (closed loop) |
| `pwm_limit` | 255 | PWM clamp |
| `ff_gain` | 100 | Open loop: PWM per rad/s (tune on the bench) |
| `encoder_timeout_s` | 0.25 | Max encoder data age before 0 PWM |
| `encoder_wait_s` | 10 | Startup wait for the first encoder message |
| `runaway_time_s`, `stall_time_s`, `max_jump_deg` | 0.5, 1.0, 15 | Fault thresholds (0 disables a check) |

The trajectory controller's own PID gains are in `config/ros2_controllers_real.yaml`.

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/arm_target_angles` | `Float32MultiArray` | Plugin → `arm.py`: PWM per motor (−255…255), despite the name |
| `/enc_angles` | `Float32MultiArray` | `arm.py` → plugin: motor angles in degrees |
| `/enc_counts` | `Int32MultiArray` | `arm.py`: raw encoder counts |
| `/arm_esp_uptime` | `UInt32` | `arm.py` → plugin: ESP32 uptime (reset detection) |
| `/arm_hardware/clear_faults` | `Empty` | Clear latched faults |
| `/arm_control_mode` | `String` | `MANUAL` / `JOYSTICK_ONLY` / `AUTONOMOUS` |
| `/arm_state`, `/input_space` | `Bool` | Mode flags (autonomous, position space) |

Motor order everywhere: base, shoulder, elbow, bevel 1, bevel 2, gripper.

### Logs

Every safety event is logged once when it starts and when it ends, with reminders while it
lasts (e.g. `ENCODERS UNPLUGGED ... setting PWM = 0`, `ENCODERS RECONNECTED`,
`JOYSTICK SIGNAL LOST`, `RUNAWAY FAULT on [base] ...`). For live PWM output add
`--ros-args --log-level RoverArmHardware:=debug`.

### First power-on checklist

Do this with the arm lifted and the kill switch in reach.

1. Start in open loop (`open_loop_joints:=all`) on the slowest speed tier.
2. For each motor: positive command turns the joint in the URDF-positive direction
   (`dir_dir[]` in `arm.c`), and `/enc_angles` **increases** (`enc_dir[]` in `arm.c`).
3. Moving wrist pitch only makes both bevel angles change by the same amount with the
   same sign; roll only makes them change with opposite signs.
4. Find the PWM below which motors don't move and tune `ff_gain`.
5. Check for lost encoder counts: move a joint fast, return it slowly, compare angles.
6. Only then switch to closed loop with low gains and tune `kp`/`ki`, then the JTC gains.

## AVATAR files

`avatar_files/` holds copies of the two AVATAR files the real arm depends on. The
AVATAR repo (`TeamAnveshak/AVATAR`) is where they are run from and should be kept in sync.

| File | Runs on | Purpose |
|---|---|---|
| `arm.c` | ESP32-S3 | Applies PWM from fields 2–7, reads the six quadrature encoders, streams PWM echo (12–17), counts (22–27), angles in degrees (32–37) and uptime (40) at 100 Hz, 1 s command watchdog |
| `arm.py` | Orin | Bridges `/arm_target_angles` → ESP32 and ESP32 → `/enc_angles`, `/enc_counts`, `/arm_target_echo`, `/arm_esp_uptime` |

`arm.c` has not yet been compiled or tested on hardware. The serial port in `arm.py` is
fixed to `/dev/ttyACM0`.

## Known issues

- `rover_arm_description/launch/rviz.launch.py` points at `rover_test_version.xacro`, which doesn't exist.
- `rover_arm_moveit_config`: the demo `ros2_controllers.yaml` names the controller `joint_trajectory_controller` while MoveIt expects `arm_controller`; `rover_arm.urdf.xacro` adds a second `ros2_control` block on top of the sim one; the SRDF virtual joint conflicts with the `world` link in the sim URDF.
- KDL IK often fails for pose goals on this 5-DOF arm; consider `pick_ik` or position-only IK.
- The three arm xacros are copies of each other and have drifted (e.g. joint limits in `rover_arm_raw.xacro`).

## URDF

There are three main URDFs of the rover's arm:
* __rover_arm_raw.xacro__: the playground for URDF changes; the raw export from LinkForge (more on that below).
* __rover_arm_sim.xacro__: used in the Gazebo simulation, with its Gazebo plugins and sensors.
* __rover_arm_real.xacro__: used on the actual rover, with the `RoverArmHardware` `ros2_control` block.

The visuals for all these xacro files are stored in the `meshes` folder within this directory, which should also be the default export location of LinkForge.

## Editing the URDF
To edit the URDF of the rover arm or to add new features such as sensors, it must be done using LinkForge, which is a blender plugin.

After LinkForge is installed, follow the steps below to make your own URDFs or edit the current one.
* __Step 1__: Pester some mech person to give you whatever parts step file.
* __Step 2__: Open this step file in Fusion 360 and rearrange all the components and bodies, such that all that remain in the tree structure are the actual links connected via revolute joints.
* __Step 3__: Export these as __binary__ STL files.
* __Step 4__: Open blender and import the STL files one by one, while arranging them as the joints should be. Use the 3D cursor to correct the object origins where you want them to be (Ask gemini for steps).
* __Step 5__: Click `N` to open the sidebar in blender, where you can access LinkForge. Click on one of the links and click `Create Link from Mesh`, which will make the link. Once it is a link, you can also create the collision, which should usually be kept as the bounding box option.
* __Step 6__: To create joints, click on the __child__ link and scroll down in the LinkForge side bar to see the `Create Joint` option, which will allow you to add the joint.
* __Step 7__: To create a sensor, click on whichever link you want the sensor to be at and click `Create sensor`, this will immediately add the required Gazebo tags in your URDF, though you may have to make further corrections, as this feature assumes a wrong coordinate frame.
