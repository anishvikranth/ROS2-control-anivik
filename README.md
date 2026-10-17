# Rover Arm – ROS 2 Control, MoveIt & Simulation

ROS 2 (Jazzy) workspace for Team Anveshak's rover arm (IRC 2027): robot description,
Gazebo simulation, MoveIt configuration, and a `ros2_control` hardware plugin that drives
the real arm through the AVATAR serial link to the ESP32.

The same controllers and MoveIt setup run in simulation and on the real arm; only the
hardware layer underneath changes.

```
 Joystick (/joy_arm) ──► real_arm_teleop_node ──┐   arm_controller + gripper_controller (JTC)
                                                ├──► or joystick_only_controller (velocity)
 MoveIt (move_group) ───────────────────────────┘              │
                                                               ▼
                         Gazebo (gz_ros2_control)   or   RoverArmHardware plugin
                         or virtual_esp.py (sim)               │ /arm_target_angles (PWM)
                                                               ▼        ▲ /enc_angles (deg)
                                                            arm.py ◄──► ESP32 (arm.c)
```

## What it can do

- **Three control modes** -- MANUAL (joystick moves joint targets), JOYSTICK_ONLY (velocity,
  works without encoders) and AUTONOMOUS (MoveIt) -- switchable from the joystick.
- **Switching between joint space control and position space control** -- button B toggles
  between moving individual joints and moving the gripper in straight lines
  (forward/left/up/pitch/roll).
- **Named poses** -- LB + A/B/X/Y sends the arm to stow / panel_ready / cache_drop /
  ground_pick; LB + Start records new ones. **Gripper presets** on RB + A/B with a
  force-limited squeeze.
- **Safety** -- encoder and joystick watchdogs, RUNAWAY / STALL / JUMP / REBOOT fault
  detection, trajectory tolerances, ESP32 command watchdog, every event logged.
- **Open-loop fallback** -- drive any joint without its encoder (`open_loop_joints:=...`).
- **Real-arm stack in simulation** -- the exact real software against Gazebo through a
  virtual ESP32, with failure drills.
- **Bring-up tool** -- `arm_bringup.py` checks motor/encoder directions and the wrist,
  and measures deadband and speed on power-on day.

## Quick start

```bash
# build
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash

# drive the real-arm software in simulation (Gazebo + RViz, joystick on /joy_arm)
ros2 launch rover_arm_description arm_real_sim.launch.py

# the real arm (ESP32 flashed with avatar_files/arm.c, arm.py available in ~/AVATAR)
ros2 launch rover_arm_description arm_real.launch.py
```

First time on real hardware? Follow the [First power-on checklist](#first-power-on-checklist).

## Contents

- [Repository layout](#repository-layout) · [Build](#build) · [Simulation](#simulation) ·
  [Real-arm stack in simulation](#real-arm-stack-in-simulation)
- Real arm: [Bring-up](#bring-up) · [Control modes](#control-modes) ·
  [Joystick mapping](#joystick-mapping-joy_arm) · [Task-space jogging](#task-space-jogging-button-b) ·
  [Named poses and the gripper](#named-poses-and-the-gripper) · [Safety behaviour](#safety-behaviour) ·
  [Hardware parameters](#hardware-parameters-rover_arm_realxacro) · [Topics](#topics) · [Logs](#logs)
- Power-on: [First power-on checklist](#first-power-on-checklist) · [Bring-up tool](#bring-up-tool)
- [AVATAR files](#avatar-files) · [Known issues](#known-issues) · [URDF](#urdf) ·
  [Editing the URDF](#editing-the-urdf)

## Repository layout

| Path | What it is |
|---|---|
| `src/rover_arm_description` | Arm URDFs (`rover_arm_raw/sim/real.xacro`), meshes, controller configs, launch files and nodes (below) |
| `  launch/arm_real.launch.py` | Real arm: `RoverArmHardware`, controllers, teleop, MoveIt, RViz, `arm.py` |
| `  launch/arm_real_sim.launch.py` | Same stack against Gazebo via `virtual_esp.py` |
| `  launch/gazebo.launch.py`, `arm.launch.py` | Quick Gazebo sim (position control), optionally with MoveIt |
| `  src/real_arm_teleop_node.cpp` | Joystick teleop: modes, task-space jogging, named poses, gripper presets |
| `  scripts/virtual_esp.py` | Simulated ESP32 + motors for `arm_real_sim` (with failure drills) |
| `  scripts/arm_bringup.py` | Power-on check tool |
| `  scripts/planning_scene_setup.py` | Adds a collision floor to MoveIt's planning scene |
| `  config/ros2_controllers_real.yaml` | Real-arm controllers, gains, tolerances |
| `src/rover_arm_hardware` | `ros2_control` SystemInterface plugin for the real arm (`RoverArmHardware`) |
| `src/rover_arm_moveit_config` | MoveIt config: SRDF (groups, **named poses**), kinematics, joint limits, RViz config |
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
| `ros2 launch rover_arm_description arm_real_sim.launch.py` | **The real-arm software stack against Gazebo** -- see [Real-arm stack in simulation](#real-arm-stack-in-simulation) |
| `ros2 launch rover_arm_description move_command.launch.py` | Sends one scripted MoveIt goal (`moveit_goal_node`). Add `use_sim_time:=false` outside Gazebo |
| `ros2 launch rover_arm_description rviz.launch.py model:=rover_arm_sim.xacro` | RViz + joint sliders to check a URDF (`model` is any xacro in `urdf/`) |
| `ros2 launch rover_arm_moveit_config demo.launch.py` | MoveIt with mock hardware (no Gazebo) |
| `ros2 launch rover_description gz.launch.py` | Full rover in Gazebo |

`gazebo.launch.py` / `arm.launch.py` drive the simulated arm through **position** interfaces
(a quick sim). `arm_real_sim.launch.py` runs the exact real-arm stack instead.

### Real-arm stack in simulation

```bash
ros2 launch rover_arm_description arm_real_sim.launch.py                         # closed loop
ros2 launch rover_arm_description arm_real_sim.launch.py open_loop_joints:=all   # no "encoders"
```

```
 Gazebo arm (velocity joints, its own controller manager in /gz)
     ^ joint velocities           | /gz/joint_states
 scripts/virtual_esp.py   -- stands in for the ESP32 + arm.py
     ^ /arm_target_angles (PWM)   | /enc_angles, /arm_esp_uptime
 arm_real.launch.py  (RoverArmHardware, controllers, teleop, MoveIt) -- unchanged
```

`virtual_esp.py` models the motors (PWM deadband, top speed, lag, wiring direction), the
differential wrist and shared gripper motor, the 1 s watchdog and 100 Hz telemetry. Use it
to practise driving, try the modes, rough-tune gains and rehearse failures before using the
real arm.

| Argument | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo window |
| `rviz` | `true` | MoveIt RViz |
| `use_joy` | `true` | Start `joy_node` publishing on `/joy_arm` |
| `open_loop_joints` | `none` | As for the real arm |
| `motor_deadband` | `30` | Virtual motors: PWM below which they don't move |
| `pwm_deadband` | `30` | `RoverArmHardware` deadband compensation |
| `ff_gain` | `255` | `RoverArmHardware` feed-forward (the virtual motors reach 1 rad/s at full PWM) |
| `spawn_cube` | `true` | Target cube in front of the arm |
| `real_stack` | `true` | `false` = Gazebo + virtual ESP only (to practise with `arm_bringup.py`) |

Failure drills while it runs:

```bash
ros2 topic pub --once /virtual_esp/unplug_encoders std_msgs/msg/Bool "{data: true}"   # false to replug
ros2 topic pub --once /virtual_esp/reboot std_msgs/msg/Empty                          # ESP32 reset
ros2 topic pub --once /virtual_esp/freeze_encoder std_msgs/msg/Int32 "{data: 0}"      # motor index, -1 = none
ros2 param set /virtual_esp enc_dir "[-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"                 # reversed encoder
```

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
| `serial_port` | `/dev/ttyACM0` | ESP32 serial port, passed to `arm.py` |
| `ff_gain` | `100.0` | PWM per rad/s (open loop, and closed-loop feed-forward) |
| `pwm_deadband` | `0.0` | Motor deadband compensation (0 = off) |
| `rviz` | `true` | Start MoveIt RViz (`false` on a headless Orin) |
| `start_arm_py` | `true` | Start `arm.py` (`arm_real_sim` sets `false`) |
| `collision_floor_z` | `none` | Add a floor at this height (m, world frame) to MoveIt's planning scene. `arm_real_sim` sets Gazebo's floor (-0.1). On the rover the deck/body should come from the rover model instead |
| `avatar_repo_path` | `~/AVATAR` | Where `orin_side/arm.py` is run from |
| `time_from_start_s` | `0.15` | Trajectory point time used by the teleop node |
| `cooldown_s` | `0.3` | Debounce for the mode buttons |

`arm_real.launch.py` starts `arm.py`, `robot_state_publisher`, `ros2_control_node` with
`RoverArmHardware`, the controllers, the teleop node, `move_group` and MoveIt RViz.

### Control modes

| Mode | Controller | How the joystick drives the arm | Needs encoders |
|---|---|---|---|
| `MANUAL` | `arm_controller` + `gripper_controller` (JointTrajectoryControllers) | Moves target joint **positions** | Yes |
| `JOYSTICK_ONLY` | `joystick_only_controller` (JointGroupVelocityController) | Sends joint **velocities** straight to the motors, no position loop | No |
| `AUTONOMOUS` | `arm_controller` + `gripper_controller`, driven by MoveIt | Joystick ignored (except the mode button) | Yes |

Pressing **A** to enter AUTONOMOUS also points RViz's MotionPlanning start and goal state at
the arm's real pose. RViz doesn't re-sync the planning group's start state by itself (only
after an execution), so without this, **Plan** after joystick moves would start from where
the last MoveIt execution ended and execution would fail. This needs **External Comm.**
ticked in the Planning tab -- `moveit.rviz` turns it on; don't save the config with it off.

The current mode is published on `/arm_control_mode`. Only one of the two controllers is
active at a time; the teleop node switches them through the controller manager.

### Joystick mapping (`/joy_arm`)

| Input | Action |
|---|---|
| `axes[0]` / `axes[1]` / `axes[4]` | Base / shoulder / elbow |
| `axes[7]` / `axes[6]` | Wrist pitch / palm roll |
| `axes[3]` | Gripper open / close |
| `button[0]` | Toggle MANUAL ↔ AUTONOMOUS |
| `button[1]` (B) | Toggle **joint space** ↔ **task space** (sticks move the gripper, see below) |
| `button[2]` | Toggle MANUAL ↔ JOYSTICK_ONLY (param `joystick_only_button_idx`) |
| `button[6]` / `button[7]` | Speed tier down / up (position step 0.005/0.01/0.02 rad, velocity 0.1/0.25/0.5 rad/s) |

| **LB** + A / B / X / Y | Go to pose `stow` / `panel_ready` / `cache_drop` / `ground_pick` (MANUAL mode) |
| **LB** + Start | Print the current arm pose as an SRDF `<group_state>` block |
| **RB** + A / B | Gripper open / closed |

Mode buttons toggle once per press; holding a button does not repeat. While LB or RB is
held, A/B/X/Y/Start do **not** do their normal job.

### Task-space jogging (button B)

In MANUAL mode, **B** switches the sticks from moving joints to moving the **gripper**:

| Input | Gripper moves |
|---|---|
| left stick up / down | forward / back (along the direction the arm points) |
| left stick left / right | left / right |
| right stick up / down | up / down |
| d-pad up / down | pitch (tilt up / down) |
| d-pad left / right | roll |

Directions you are not commanding are held: pushing forward slides the gripper straight
forward and keeps it level -- what you want for pressing buttons, pulling drawers or
inserting the plug. The speed tier buttons set the speed (0.02 / 0.05 / 0.1 m/s and
0.1 / 0.25 / 0.5 rad/s). Press **B** again for joint mode. Named poses (LB) and the gripper
(RB) work in both modes.

Things to know:
- The arm has 5 joints, so it can control position + pitch + roll but **not yaw** -- the
  gripper can't turn left/right on its own (the whole arm swings with the base).
- Near the edge of its reach (arm stretched straight) or at a joint limit, some directions
  are impossible. The arm then slows down and the terminal says *"can't move that way from
  here"* instead of drifting somewhere you didn't ask for. Start from a bent-elbow pose
  (e.g. `ground_pick` or `panel_ready`), not the straight-out zero pose.
- With `collision_floor_z` set (the sim does), the gripper stops 8 cm above that floor.
- Needs closed loop (encoders); not available in JOYSTICK_ONLY mode.
- Implementation: the 6x5 Jacobian at the tool point (`tool_offset`, 0.17 m ahead of the
  palm) is projected onto the 5 controllable directions and solved with damped least squares,
  scaled to the joint speed limits. Tuning parameters (`task_linear_levels`,
  `task_angular_levels`, `task_damping`, `task_max_joint_speed`, `tool_offset`) are on
  `real_arm_teleop_node`.

### Named poses and the gripper

The arm can jump to saved poses and the gripper can open/close with one button combo.

**1. Start the arm** (sim or real) and check the terminal shows:

```
Named poses: LB+A=stow, LB+B=panel_ready, LB+X=cache_drop, LB+Y=ground_pick. Gripper presets: RB+A=open, RB+B=closed.
```

**2. Be in MANUAL mode** -- the normal joystick mode at startup. Not AUTONOMOUS (press A
again to leave it) and not JOYSTICK_ONLY (press button 2 again). The terminal warns if you
try a pose in the wrong mode.

**3. Move to a pose: hold LB, tap a face button.**

| Hold LB + | Pose | Use |
|---|---|---|
| A | `stow` | folded up, for driving |
| B | `panel_ready` | forward and raised, for the IDMO panel |
| X | `cache_drop` | to the side -- **placeholder**, record your own (step 5) |
| Y | `ground_pick` | forward, pointing down, for picking up objects |

The arm moves smoothly in joint space (the joint with furthest to go at `pose_speed`,
0.3 rad/s) and the terminal prints `Moving to pose ...` / `Reached pose ...`.
**Touch any arm stick to cancel** -- the arm stops where it is and normal joystick control
continues. While LB is held, A/B/X/Y only select poses, so you can't switch mode by accident.

**4. Gripper: hold RB, tap A (open) or B (close).** The gripper stick cancels. Closing on
an object keeps squeezing with a capped force (see below), so it neither drops the object
nor burns the motor.

**5. Save your own poses.** The shipped values are starting guesses from forward kinematics.
1. Drive the arm with the joystick to exactly the pose you want.
2. Hold **LB** and tap **Start**. The terminal prints a ready-made block:
   ```
   <group_state name="NEW_POSE" group="arm">
       <joint name="base_link_joint" value="1.234"/>
       ...
   </group_state>
   ```
3. In `src/rover_arm_moveit_config/config/rover_arm.srdf`, replace the numbers of the pose
   you want to change (e.g. `cache_drop`) with the printed ones, keeping its name.
4. Restart the launch (no rebuild needed with `--symlink-install`).

The same poses appear in RViz's MotionPlanning goal list, so MoveIt can use them in
AUTONOMOUS mode too -- the SRDF is the single place they are defined.

**If a combo does nothing**, your controller may number its buttons differently (the
defaults assume an Xbox-style pad: LB = 4, RB = 5, Start = 7). Check with
`ros2 topic echo /joy_arm` while pressing the button, then set the teleop node's
`pose_modifier_button`, `gripper_modifier_button` and `pose_record_button` parameters.

**How the gripper works underneath.** The fingers have their own `gripper_controller`, so
MoveIt can command the `arm` and `gripper` groups separately. Closing on an object stalls
the gripper motor by design, so: the gripper is exempt from the STALL fault, its PWM is
capped at `gripper_pwm_limit` (180 by default -- this sets the grip force), and
`gripper_controller` has no tolerances, so it keeps squeezing instead of giving up when the
fingers stop short of the target.

### Safety behaviour

| Situation | What happens |
|---|---|
| No `/enc_angles` at startup (closed-loop motors) | The plugin waits `encoder_wait_s`, sending 0 PWM to wake the ESP32; if nothing arrives, closed-loop motors stay disabled (0 PWM) until restart |
| Encoders stop mid-run | Closed-loop motors get 0 PWM after `encoder_timeout_s`; resumes when data returns |
| A joint falls > 0.25 rad behind its trajectory (e.g. during an encoder dropout) | `arm_controller` aborts the trajectory and holds the current position, so the arm doesn't lurch toward the old target afterwards |
| Joystick link lost (> 0.5 s) | JOYSTICK_ONLY: velocities forced to 0. MANUAL: arm holds the last target |
| USB serial to the ESP32 lost | `arm.py` drops commands and reconnects every second; an ESP32 reset seen after reconnecting is logged and raises a REBOOT fault |
| No command reaches the ESP32 for 1 s | ESP32 firmware stops all motors (LED red) |
| Hardware deactivated | 0 PWM sent to all motors |

**Faults** (closed-loop motors). A fault latches the motor at 0 PWM; the two bevel motors
fault together because the differential wrist needs both.

| Fault | Trigger | Usual cause |
|---|---|---|
| `RUNAWAY` | Driven one way but moving the other way for `runaway_time_s` | Encoder direction reversed (fix `enc_dir[]` / `dir_dir[]` in `arm.c`) |
| `STALL` | Move commanded, ≥ 50 % PWM, not moving for `stall_time_s` (not the gripper, see above) | Joint blocked, encoder not counting, motor/driver failure |
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
| `kp`, `ki` | 40, 5 | Velocity PI correction gains (closed loop) |
| `pwm_limit` | 255 | PWM clamp |
| `ff_gain` | 100 | PWM per rad/s: the whole command in open loop, the feed-forward in closed loop (PWM = ff_gain x velocity + PI correction). Tune in open loop first |
| `pwm_deadband` | 0 (off) | PWM below which the motors don't move; small commands are scaled up past it while a motion is commanded |
| `gripper_pwm_limit` | 180 | Gripper PWM cap = grip force on an object |
| `gripper_stall_fault` | 0 | 1 = also apply the STALL fault to the gripper |
| `encoder_timeout_s` | 0.25 | Max encoder data age before 0 PWM |
| `encoder_wait_s` | 10 | Startup wait for the first encoder message |
| `runaway_time_s`, `stall_time_s`, `max_jump_deg` | 0.5, 1.0, 15 | Fault thresholds (0 disables a check) |

`ff_gain` and `pwm_deadband` can also be set from the launch file without editing the xacro.
The trajectory controller passes the trajectory's own velocity through
(`ff_velocity_scale: 1.0`); without it a joint must lag behind to move at all.
MoveIt's speed limits (`rover_arm_moveit_config/config/joint_limits.yaml`) must not exceed
what the motors can do: 1.0 rad/s per joint, and 0.5 rad/s for wrist pitch and palm roll
because they share the two bevel motors. On the real arm set them from `arm_bringup.py`.
The trajectory controller's PID gains and tracking tolerances (`constraints`) are in
`config/ros2_controllers_real.yaml`. The tolerances are deliberately loose while the gains
are untuned; tighten them after tuning.

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

1. Flash `arm.c`, start **only** `arm.py` (not `arm_real.launch.py`), with the arm in its
   URDF zero pose.
2. Run the bring-up tool (below). It checks encoder data, motor/encoder directions, the
   differential wrist, and measures deadband and speed.
3. Apply what it reports: fix `enc_dir[]` / `dir_dir[]` in `arm.c` and re-flash; re-run the
   tool until everything is OK.
4. Check for lost encoder counts: move a joint fast, return it slowly, compare angles.
5. Start `arm_real.launch.py open_loop_joints:=all pwm_deadband:=<value> ff_gain:=<value>`
   on the slowest speed tier and drive each joint.
6. Only then run closed loop (`arm_real.launch.py pwm_deadband:=... ff_gain:=...`) and tune
   `kp`/`ki`, then the JTC gains.

### Bring-up tool

```bash
# with only arm.py running (or: arm_real_sim.launch.py real_stack:=false to practise)
ros2 run rover_arm_description arm_bringup.py                      # interactive, all motors
ros2 run rover_arm_description arm_bringup.py --motors base,elbow   # subset
ros2 run rover_arm_description arm_bringup.py --yes                 # no questions
```

It talks straight to `arm.py` (PWM on `/arm_target_angles`, angles on `/enc_angles`), so it
doesn't depend on any `RoverArmHardware` setting, and refuses to run if anything else is
publishing PWM. For each motor it:

- ramps PWM slowly in both directions until the encoder moves -> **deadband**, and whether
  the encoder counts the same way as the PWM (interactively it also asks whether the joint
  moved in the URDF-positive direction, which separates a reversed motor from a reversed encoder);
- measures speed at two PWM levels -> fitted **deadband** and top speed -> **`ff_gain`**;
- for the wrist, drives both bevels together (pitch) and opposite (roll) and checks both
  encoders respond as expected.

It ends with a summary -- a ready-to-paste `enc_dir[]` line if any encoder is reversed,
suggested `pwm_deadband` / `ff_gain`, and the launch command to try -- and saves a report to
`~/arm_bringup_reports/`.

Safety: PWM capped at `--max-pwm` (120), every move stops after `--max-travel-deg` (10 deg)
and is reversed, interactive mode asks before each motor, Ctrl-C sends 0 PWM.

## AVATAR files

`avatar_files/` holds copies of the two AVATAR files the real arm depends on. The
AVATAR repo (`TeamAnveshak/AVATAR`) is where they are run from and should be kept in sync.

| File | Runs on | Purpose |
|---|---|---|
| `arm.c` | ESP32-S3 | Applies PWM from fields 2–7, reads the six quadrature encoders, streams PWM echo (12–17), counts (22–27), angles in degrees (32–37) and uptime (40) at 100 Hz, 1 s command watchdog |
| `arm.py` | Orin | Bridges `/arm_target_angles` → ESP32 and ESP32 → `/enc_angles`, `/enc_counts`, `/arm_target_echo`, `/arm_esp_uptime`. ROS params `port` (default `/dev/ttyACM0`) and `baud`; reconnects automatically if the port is lost |

`arm.c` compiles for the ESP32-S3 (esp32 board package 3.3.12) but has not been tested on
hardware yet. `arm.py` has been tested against a simulated serial port only.

### Compiling and flashing on Linux

The Arduino tools need the sketch as `<name>/<name>.ino`, so copy `arm.c` to `arm/arm.ino`.

```bash
# One-time setup
arduino-cli config init
arduino-cli config set board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.3.12
# The AVATAR Arduino library must be in ~/Arduino/libraries/AVATAR
sudo usermod -aG dialout $USER     # then log out and back in

# Compile / flash (close arm.py first -- only one program can hold the port)
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc arm
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc --upload -p /dev/ttyACM0 arm
```

`CDCOnBoot=cdc` (USB CDC On Boot: Enabled) is required for `arm.py` to talk to the board
over `/dev/ttyACM0`. Match any other board settings (flash size, PSRAM, USB mode) to the
ones used in the Arduino IDE.

## Known issues

- `gazebo.launch.py` / `arm.launch.py` (quick position-control sim) were updated for the
  separate gripper controller but not re-tested since; `arm_real_sim.launch.py` is tested.
- KDL IK often fails for pose goals on this 5-DOF arm; consider `pick_ik` or position-only IK.
- MoveIt only knows the arm (and, in sim, the floor). Until the arm is on the rover model, nothing stops it planning through the rover body.
- The three arm xacros are copies of each other and have drifted (e.g. joint limits in `rover_arm_raw.xacro`).
- `move_group` logs errors about `DepthImageOctomapUpdater`: `moveit_ros_perception` isn't installed. Harmless until depth-camera obstacle avoidance is needed; then install it or drop `sensors_3d.yaml`.
- `rover_arm_raw.xacro` uses relative mesh paths from LinkForge, so it can't be shown in RViz directly.
- While a motor is faulted, holding the joystick makes `arm_controller` log "Holding position due to state tolerance violation" for every new target (safe, just noisy).

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
