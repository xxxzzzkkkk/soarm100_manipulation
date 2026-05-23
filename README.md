# soarm100_manipulation

This package contains the C++ application layer for SO-ARM100:

- Xbox controller teleoperation for the real arm
- A small `RobotInterface` wrapper around the arm and gripper controllers
- A Pinocchio position-IK example

The package does not talk to the servos directly. Real hardware is started by
`so_arm_100_bringup` / `so_arm_100_hardware`, and this package sends commands to
the already-running ROS 2 controllers.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select soarm100_manipulation
source install/setup.bash
```

## Xbox Joint Teleop

The Xbox teleop node reads `/joy`, reads the current robot state from
`/joint_states`, then sends commands to:

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/gripper_cmd
```

Run everything together:

```bash
ros2 launch soarm100_manipulation xbox_joint_teleop.launch.py \
  start_hardware:=true \
  start_joy:=true \
  rviz:=true \
  serial_port:=/dev/ttyACM0 \
  device_id:=0
```

If the Xbox controller is `/dev/input/js1`, use:

```bash
device_id:=1
```

### Controls

```text
Left stick X      Shoulder_Rotation
Left stick Y      Shoulder_Pitch
Right stick Y     Elbow
Right stick X     Wrist_Pitch
LB / RB           Wrist_Roll
LT / RT           Gripper
```

### Useful Parameters

`max_joint_velocity` controls how fast the arm jogs in rad/s. Increase it if the
arm feels too slow:

```bash
max_joint_velocity:=0.6
```

`command_period` controls how often the teleop node sends a new command:

```bash
command_period:=0.06
```

`trajectory_duration` controls the duration of each small trajectory goal:

```bash
trajectory_duration:=0.12
```

`min_goal_delta` filters tiny target changes:

```bash
min_goal_delta:=0.001
```

`axis_deadzone` filters small joystick noise:

```bash
axis_deadzone:=0.15
```

`log_commands` prints throttled arm/gripper targets:

```bash
log_commands:=true
```

Example with a faster arm:

```bash
ros2 launch soarm100_manipulation xbox_joint_teleop.launch.py \
  start_hardware:=true \
  start_joy:=true \
  rviz:=true \
  serial_port:=/dev/ttyACM0 \
  device_id:=0 \
  max_joint_velocity:=0.6 \
  command_period:=0.06 \
  trajectory_duration:=0.12 \
  min_goal_delta:=0.001
```

## Checks

Check the controller stack:

```bash
ros2 control list_controllers
ros2 action info /arm_controller/follow_joint_trajectory
ros2 action info /gripper_controller/gripper_cmd
```

Check robot state:

```bash
ros2 topic echo /joint_states --once
```

Check Xbox input:

```bash
ros2 topic echo /joy --once
```

If `/joy` is empty, try another joystick device id:

```bash
ros2 launch soarm100_manipulation xbox_joint_teleop.launch.py device_id:=1
```

## RobotInterface

`RobotInterface` is a small C++ wrapper for direct controller actions. It uses:

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/gripper_cmd
/joint_states
```

The arm joint order is:

```text
Shoulder_Rotation
Shoulder_Pitch
Elbow
Wrist_Pitch
Wrist_Roll
```

Main methods:

```cpp
bool waitForServers(double timeout_sec = 5.0);
bool moveJoints(const std::vector<double>& positions, double duration_sec = 3.0);
bool moveGripper(double position, double timeout_sec = 3.0);
bool openGripper();
bool closeGripper();
bool goHome();
std::vector<double> getCurrentArmPositions() const;
double getCurrentGripperPosition() const;
```

## Pinocchio IK

`pinocchio_ik` is a learning/demo node for position-only IK. It loads the URDF
into Pinocchio, reads `/joint_states` as the initial configuration, solves for an
end-effector xyz target, and can optionally send the solved joint target to the
arm controller.

Run without moving the robot:

```bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py
```

Execute the solved target after hardware is running:

```bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py execute:=true
```

Use an offset from the current end-effector position:

```bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py \
  use_offset:=true \
  dx:=0.02 \
  dy:=0.0 \
  dz:=0.0 \
  execute:=true
```
