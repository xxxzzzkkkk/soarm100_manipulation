# soarm100_manipulation

This package is the application and algorithm layer for SO-ARM100. It sits above
`ros2_control` and uses the already running controllers instead of talking to the
servos directly.

## RobotInterface

`RobotInterface` is a C++ execution API for the real SO-ARM100 arm.

It connects to:

```text
/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/gripper_controller/gripper_cmd [control_msgs/action/ParallelGripperCommand]
/joint_states [sensor_msgs/msg/JointState]
```

It provides:

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

The arm joint order for `moveJoints()` is:

```text
Shoulder_Rotation
Shoulder_Pitch
Elbow
Wrist_Pitch
Wrist_Roll
```

`RobotInterface` does not start controllers. The real hardware stack must already
be running through `so_arm_100_bringup hardware.launch.py`.

## MotionPlanner

`MotionPlanner` is a MoveIt2 planning wrapper for SO-ARM100.

It uses:

```cpp
moveit::planning_interface::MoveGroupInterface
```

It provides:

```cpp
bool moveToNamedTarget(const std::string& target_name);
bool moveToJointTarget(const std::vector<double>& joint_positions);
bool moveToPoseTarget(const geometry_msgs::msg::PoseStamped& target_pose);
bool planToPoseTarget(const geometry_msgs::msg::PoseStamped& target_pose, Plan& plan);
bool executePlan(const Plan& plan);
geometry_msgs::msg::PoseStamped getCurrentPose();
```

The default MoveIt planning group is:

```text
arm
```

`MotionPlanner` is responsible for pose goals, MoveIt2 IK, planning, collision
checking, and execution through MoveIt. `RobotInterface` remains useful for
direct controller-action smoke tests and gripper control.

## Move-To-Pose Demo

`demo_move_to_pose` is a MoveIt2 pose planning demo. It reads the current end
effector pose, applies a small Cartesian offset, plans with MoveIt2, and executes
the resulting trajectory.

Sequence:

```text
connect to MoveIt2 move_group
read current end-effector pose
target pose = current pose + Cartesian offset
plan with MoveIt2
execute planned trajectory
```

## Pinocchio IK

`pinocchio_ik` is my hand-written Pinocchio IK example for learning the basic
kinematics pipeline.

I intentionally solve **position-only IK** here:

```text
target xyz = x/y/z
```

I do not solve full 6D position + orientation IK in this node because the
SO-ARM100 arm has 5 arm joints:

```text
Shoulder_Rotation
Shoulder_Pitch
Elbow
Wrist_Pitch
Wrist_Roll
```

A full end-effector pose is a 6D task: 3D position plus 3D orientation. With a
5-DoF arm, that task is generally over-constrained. So in this learning node I
keep the math honest: I use Pinocchio FK and the linear part of the end-effector
Jacobian to solve xyz, and I leave orientation unconstrained.

The internal sequence is:

```text
load URDF into Pinocchio Model
read /joint_states as the initial q
compute current end-effector frame position
target position = x/y/z
iterate FK -> position error -> frame Jacobian -> damped least-squares dq
optionally send the solved 5 arm joint positions to /arm_controller/follow_joint_trajectory
```

First run it without moving the robot:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py x:=0.02 y:=-0.3888 z:=0.2368
```

To execute the solved joint target, start Gazebo or the real ros2_control stack
first, then run:

```bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py x:=0.02 y:=-0.3888 z:=0.2368 execute:=true
```

For quick relative-motion tests, I kept an offset mode:

```bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py use_offset:=true dx:=0.02 execute:=true
```

For this controller test, Gazebo is the better target than plain RViz. RViz is
good for visualization, TF, and MoveIt planning views, but it is not a physics
or controller execution environment by itself. Gazebo runs the ros2_control
controller stack, so `/arm_controller/follow_joint_trajectory` can actually
accept and execute the trajectory.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select soarm100_manipulation
source install/setup.bash
```

## Run On Real Hardware

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so_arm_100_bringup hardware.launch.py serial_port:=/dev/ttyACM0
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch soarm100_manipulation demo_move_to_pose.launch.py
```

The launch file starts `move_group` and then runs the demo after a short delay.
Use parameters to change the Cartesian offset:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch soarm100_manipulation demo_move_to_pose.launch.py dx:=0.02 dy:=0.0 dz:=0.02
```

If `move_group` is already running, skip starting it:

```bash
ros2 launch soarm100_manipulation demo_move_to_pose.launch.py start_move_group:=false
```

## Run With RViz2

Start RViz2 with the demo launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch soarm100_manipulation demo_move_to_pose.launch.py rviz:=true
```

## Run With Gazebo Simulation

For simulation, replace the hardware launch with Gazebo, then start MoveIt2 and
run the same demo:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so_arm_100_bringup gz.launch.py
```

Then run `move_group.launch.py`, optionally `moveit_rviz.launch.py`, and
`demo_move_to_pose` in separate terminals as shown above.

For the Pinocchio IK controller path, run Gazebo first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so_arm_100_bringup gz.launch.py
```

Then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch soarm100_manipulation pinocchio_ik.launch.py x:=0.02 y:=-0.3888 z:=0.2368 execute:=true
```

Expected success markers:

```text
Starting MoveIt2 pose planning demo
Pose planning succeeded
Plan execution succeeded
```

## Useful Checks

```bash
ros2 control list_controllers
ros2 action list -t | grep controller
ros2 topic echo /joint_states
```
