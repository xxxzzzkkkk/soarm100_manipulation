#include "soarm100_manipulation/robot_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace soarm100_manipulation
{

namespace
{
// 这些名字来自运行时接口表：
// ros2 action list -t 和 ros2 topic list 可以验证它们是否存在。
constexpr const char * kArmActionName = "/arm_controller/follow_joint_trajectory";
constexpr const char * kGripperActionName = "/gripper_controller/gripper_cmd";
constexpr const char * kJointStatesTopic = "/joint_states";

// trajectory_msgs/JointTrajectoryPoint 需要 builtin_interfaces::msg::Duration，
// 这里把更容易使用的 double 秒数转换成 ROS message。
builtin_interfaces::msg::Duration secondsToDurationMsg(double seconds)
{
  builtin_interfaces::msg::Duration msg;
  msg.sec = static_cast<int32_t>(std::floor(seconds));
  msg.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(msg.sec)) * 1e9);
  return msg;
}
}  // namespace

RobotInterface::RobotInterface(const rclcpp::Node::SharedPtr & node)
: node_(node),
  // 这个顺序就是 moveJoints({q1, q2, q3, q4, q5}) 的参数顺序。
  arm_joint_names_({
    "Shoulder_Rotation",
    "Shoulder_Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
  }),
  gripper_joint_name_("Gripper"),
  home_positions_({0.0, 0.0, 0.0, 0.0, 0.0}),
  gripper_open_position_(0.7854),
  // A conservative close target for the real SO-ARM100 gripper. The SRDF close pose
  // may be below the physical/calibrated limit and can make the action abort on stall.
  gripper_closed_position_(0.0)
{
  // RobotInterface 不启动 controller，只连接已经由 controller_manager 启动的 action server。
  arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, kArmActionName);
  gripper_client_ = rclcpp_action::create_client<ParallelGripperCommand>(node_, kGripperActionName);

  // 缓存 /joint_states，供 getCurrentArmPositions()/getCurrentGripperPosition() 查询。
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    kJointStatesTopic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RobotInterface::jointStateCallback, this, std::placeholders::_1));
}

bool RobotInterface::waitForServers(double timeout_sec)
{
  return waitForArmServer(timeout_sec) && waitForGripperServer(timeout_sec);
}

bool RobotInterface::moveJoints(const std::vector<double> & positions, double duration_sec)
{
  // JointTrajectoryController 要求目标点的关节数量和 joint_names 一一对应。
  if (positions.size() != arm_joint_names_.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "moveJoints expected %zu positions, got %zu",
      arm_joint_names_.size(), positions.size());
    return false;
  }

  if (duration_sec <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "moveJoints duration must be positive");
    return false;
  }

  if (!waitForArmServer(2.0))
  {
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = arm_joint_names_;

  // 这里只发送一个目标点：让 controller 在 duration_sec 秒后到达 positions。
  // 中间插值由 joint_trajectory_controller 处理。
  trajectory_msgs::msg::JointTrajectoryPoint target_point;
  target_point.positions = positions;
  target_point.time_from_start = secondsToDurationMsg(duration_sec);
  goal.trajectory.points.push_back(target_point);

  // async_send_goal 是异步 API；这里用 spin_until_future_complete 封装成阻塞式调用，
  // 方便 demo / FSM 按顺序执行动作。
  auto goal_future = arm_client_->async_send_goal(goal);
  const auto goal_status =
    rclcpp::spin_until_future_complete(node_, goal_future, std::chrono::seconds(5));
  if (goal_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Timed out while sending arm trajectory goal");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "Arm trajectory goal was rejected");
    return false;
  }

  auto result_future = arm_client_->async_get_result(goal_handle);
  const auto wait_time = std::chrono::duration<double>(duration_sec + 5.0);
  const auto result_status = rclcpp::spin_until_future_complete(node_, result_future, wait_time);
  if (result_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for arm trajectory result");
    return false;
  }

  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Arm trajectory failed with result code %d",
      static_cast<int>(result.code));
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Arm trajectory completed");
  return true;
}

bool RobotInterface::moveGripper(double position, double timeout_sec)
{
  if (!std::isfinite(position))
  {
    RCLCPP_ERROR(node_->get_logger(), "moveGripper position must be finite");
    return false;
  }

  if (!waitForGripperServer(2.0))
  {
    return false;
  }

  // Jazzy 的 parallel_gripper_action_controller 使用 ParallelGripperCommand：
  // position 是数组，即使这里只控制一个 Gripper joint。
  ParallelGripperCommand::Goal goal;
  goal.command.name.push_back(gripper_joint_name_);
  goal.command.position.push_back(position);

  auto goal_future = gripper_client_->async_send_goal(goal);
  const auto goal_status =
    rclcpp::spin_until_future_complete(node_, goal_future, std::chrono::seconds(5));
  if (goal_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Timed out while sending gripper goal");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected");
    return false;
  }

  auto result_future = gripper_client_->async_get_result(goal_handle);
  const auto result_status = rclcpp::spin_until_future_complete(
    node_, result_future, std::chrono::duration<double>(timeout_sec));
  if (result_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper result");
    return false;
  }

  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Gripper command failed with result code %d",
      static_cast<int>(result.code));
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Gripper moved to %.3f", position);
  return true;
}

bool RobotInterface::openGripper()
{
  return moveGripper(gripper_open_position_);
}

bool RobotInterface::closeGripper()
{
  return moveGripper(gripper_closed_position_);
}

bool RobotInterface::goHome()
{
  return moveJoints(home_positions_, 3.0);
}

std::vector<double> RobotInterface::getCurrentArmPositions() const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  // 按 arm_joint_names_ 的固定顺序返回，避免 /joint_states 原始顺序变化影响上层代码。
  std::vector<double> positions;
  positions.reserve(arm_joint_names_.size());
  for (const auto & joint_name : arm_joint_names_)
  {
    const auto it = latest_joint_positions_.find(joint_name);
    if (it == latest_joint_positions_.end())
    {
      return {};
    }
    positions.push_back(it->second);
  }
  return positions;
}

double RobotInterface::getCurrentGripperPosition() const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  const auto it = latest_joint_positions_.find(gripper_joint_name_);
  if (it == latest_joint_positions_.end())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return it->second;
}

const std::vector<std::string> & RobotInterface::armJointNames() const
{
  return arm_joint_names_;
}

const std::string & RobotInterface::gripperJointName() const
{
  return gripper_joint_name_;
}

void RobotInterface::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);

  // JointState 的 name 和 position 是平行数组：name[i] 对应 position[i]。
  const auto count = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < count; ++i)
  {
    latest_joint_positions_[msg->name[i]] = msg->position[i];
  }
}

bool RobotInterface::waitForArmServer(double timeout_sec)
{
  if (arm_client_->wait_for_action_server(std::chrono::duration<double>(timeout_sec)))
  {
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Action server %s is not available", kArmActionName);
  return false;
}

bool RobotInterface::waitForGripperServer(double timeout_sec)
{
  if (gripper_client_->wait_for_action_server(std::chrono::duration<double>(timeout_sec)))
  {
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Action server %s is not available", kGripperActionName);
  return false;
}

}  // namespace soarm100_manipulation
