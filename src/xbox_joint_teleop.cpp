#include "soarm100_manipulation/xbox_joint_teleop.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

builtin_interfaces::msg::Duration secondsToDuration(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>((seconds - duration.sec) * 1e9);
  return duration;
}

}  // namespace

namespace soarm100_manipulation
{

XboxJointTeleop::XboxJointTeleop()
: Node("xbox_joint_teleop")
{
  axis_deadzone_ = declare_parameter<double>("axis_deadzone", 0.15);
  max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 0.45);
  max_gripper_velocity_ = declare_parameter<double>("max_gripper_velocity", 0.4);
  command_period_ = declare_parameter<double>("command_period", 0.06);
  trajectory_duration_ = declare_parameter<double>("trajectory_duration", 0.12);
  min_goal_delta_ = declare_parameter<double>("min_goal_delta", 0.001);
  log_commands_ = declare_parameter<bool>("log_commands", true);

  joint_limits_ = {
    {"Shoulder_Rotation", {-1.96, 1.96}},
    {"Shoulder_Pitch", {-1.745, 1.745}},
    {"Elbow", {-1.5, 1.5}},
    {"Wrist_Pitch", {-1.658, 1.658}},
    {"Wrist_Roll", {-2.75, 2.75}},
    {"Gripper", {-0.1792, 1.5708}},
  };

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(*msg); });

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy",
    rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) { latest_joy_ = *msg; });

  arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this,
    "/arm_controller/follow_joint_trajectory");

  gripper_client_ = rclcpp_action::create_client<ParallelGripperCommand>(
    this,
    "/gripper_controller/gripper_cmd");

  timer_ = create_wall_timer(
    std::chrono::duration<double>(command_period_),
    [this]() { sendCommand(); });

  RCLCPP_INFO(get_logger(), "Xbox joint teleop started");
  RCLCPP_INFO(
    get_logger(),
    "Axes: LS-x shoulder rotate, LS-y shoulder pitch, RS-y elbow, "
    "RS-x wrist pitch, LB/RB wrist roll, LT/RT gripper");
}

void XboxJointTeleop::onJointState(const sensor_msgs::msg::JointState & msg)
{
  const auto count = std::min(msg.name.size(), msg.position.size());
  for (std::size_t i = 0; i < count; ++i)
  {
    if (joint_limits_.count(msg.name[i]) > 0)
    {
      joint_positions_[msg.name[i]] = msg.position[i];
    }
  }
}

double XboxJointTeleop::axis(std::size_t index) const
{
  if (!latest_joy_ || index >= latest_joy_->axes.size())
  {
    return 0.0;
  }

  const double value = latest_joy_->axes[index];
  return std::abs(value) < axis_deadzone_ ? 0.0 : value;
}

bool XboxJointTeleop::button(std::size_t index) const
{
  return latest_joy_ && index < latest_joy_->buttons.size() &&
    latest_joy_->buttons[index] == 1;
}

double XboxJointTeleop::clampJoint(const std::string & joint, double value) const
{
  const auto limit = joint_limits_.at(joint);
  return std::clamp(value, limit.first, limit.second);
}

bool XboxJointTeleop::readArmPositions(std::vector<double> & positions) const
{
  positions.clear();
  positions.reserve(arm_joints_.size());

  for (const auto & joint : arm_joints_)
  {
    const auto it = joint_positions_.find(joint);
    if (it == joint_positions_.end())
    {
      return false;
    }
    positions.push_back(it->second);
  }
  return true;
}

void XboxJointTeleop::sendCommand()
{
  if (!latest_joy_)
  {
    return;
  }

  std::vector<double> arm_positions;
  if (!readArmPositions(arm_positions))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for /joint_states. Start hardware.launch.py or set start_hardware:=true.");
    return;
  }

  const std::array<double, 5> velocities = {
    axis(0),
    axis(1),
    axis(4),
    axis(3),
    static_cast<double>(button(5)) - static_cast<double>(button(4)),
  };

  const bool arm_active = std::any_of(
    velocities.begin(),
    velocities.end(),
    [](double value) { return std::abs(value) > 0.0; });

  if (arm_active)
  {
    if (!arm_target_positions_)
    {
      arm_target_positions_ = arm_positions;
    }

    std::vector<double> target_positions;
    target_positions.reserve(arm_joints_.size());
    for (std::size_t i = 0; i < arm_joints_.size(); ++i)
    {
      const double velocity = velocities[i] * max_joint_velocity_;
      const double target =
        (*arm_target_positions_)[i] + velocity * command_period_;
      const double clamped_target = clampJoint(arm_joints_[i], target);
      target_positions.push_back(clamped_target);
    }
    const double max_delta = [&]() {
      double value = 0.0;
      for (std::size_t i = 0; i < arm_positions.size(); ++i)
      {
        value = std::max(value, std::abs(target_positions[i] - (*arm_target_positions_)[i]));
      }
      return value;
    }();
    if (max_delta < min_goal_delta_)
    {
      return;
    }
    arm_target_positions_ = target_positions;
    sendArmGoal(target_positions);
  }
  else
  {
    arm_target_positions_ = arm_positions;
  }

  sendGripperCommand();
}

void XboxJointTeleop::sendArmGoal(const std::vector<double> & positions)
{
  if (!arm_client_->action_server_is_ready())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for /arm_controller/follow_joint_trajectory");
    return;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = arm_joints_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(arm_joints_.size(), 0.0);
  point.time_from_start = secondsToDuration(trajectory_duration_);
  goal.trajectory.points.push_back(point);

  if (log_commands_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Arm target [rad]: %.3f %.3f %.3f %.3f %.3f",
      positions[0],
      positions[1],
      positions[2],
      positions[3],
      positions[4]);
  }

  arm_client_->async_send_goal(goal);
}

void XboxJointTeleop::sendGripperCommand()
{
  const auto it = joint_positions_.find(gripper_joint_);
  if (it == joint_positions_.end())
  {
    return;
  }

  const double left_trigger = (1.0 - axis(2)) * 0.5;
  const double right_trigger = (1.0 - axis(5)) * 0.5;
  const double velocity = right_trigger - left_trigger;
  if (std::abs(velocity) < axis_deadzone_)
  {
    return;
  }

  const double target =
    clampJoint(gripper_joint_, it->second + velocity * max_gripper_velocity_ * command_period_);

  if (!gripper_client_->action_server_is_ready())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Waiting for /gripper_controller/gripper_cmd");
    return;
  }

  ParallelGripperCommand::Goal goal;
  goal.command.name = {gripper_joint_};
  goal.command.position = {target};
  goal.command.velocity = {max_gripper_velocity_};

  if (log_commands_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Gripper target [rad]: %.3f",
      target);
  }

  gripper_client_->async_send_goal(goal);
}

}  // namespace soarm100_manipulation
