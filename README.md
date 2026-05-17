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

## Demo

`demo_move_to_pose` is currently a RobotInterface smoke test. It validates the
control chain before adding MoveIt2 pose planning.

Sequence:

```text
wait for action servers
print initial joint state
goHome()
openGripper()
moveJoints({0.0, -0.30, 0.50, 0.20, 0.0}, 3.0)
closeGripper()
goHome()
print final joint state
```

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
ros2 run soarm100_manipulation demo_move_to_pose
```

Expected success markers:

```text
Arm trajectory completed
Gripper moved to 0.785
Gripper moved to 0.000
Demo completed
```

## Useful Checks

```bash
ros2 control list_controllers
ros2 action list -t | grep controller
ros2 topic echo /joint_states
```

