#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "soarm100_manipulation/pinocchio_ik_solver.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

builtin_interfaces::msg::Duration secondsToDurationMsg(double seconds)
{
  builtin_interfaces::msg::Duration msg;
  msg.sec = static_cast<int32_t>(std::floor(seconds));
  msg.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(msg.sec)) * 1e9);
  return msg;
}

// 我先等一帧 /joint_states，尽量用 Gazebo/真机当前关节角作为 IK 初值。
std::optional<sensor_msgs::msg::JointState> waitForJointState(
  const rclcpp::Node::SharedPtr & node,
  std::chrono::milliseconds timeout)
{
  std::optional<sensor_msgs::msg::JointState> latest;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SystemDefaultsQoS(),
    [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) { latest = *msg; });

  const auto deadline = Clock::now() + timeout;
  while (rclcpp::ok() && !latest && Clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  return latest;
}

bool sendArmTrajectory(
  const rclcpp::Node::SharedPtr & node,
  const std::string & action_name,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions,
  const double duration_sec,
  const double server_timeout_sec)
{
  const auto logger = node->get_logger();
  auto client = rclcpp_action::create_client<FollowJointTrajectory>(node, action_name);

  RCLCPP_INFO(logger, "Waiting for trajectory action server: %s", action_name.c_str());
  if (!client->wait_for_action_server(std::chrono::duration<double>(server_timeout_sec)))
  {
    RCLCPP_ERROR(logger, "Trajectory action server is not available: %s", action_name.c_str());
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint target_point;
  target_point.positions = positions;
  target_point.time_from_start = secondsToDurationMsg(duration_sec);
  goal.trajectory.points.push_back(target_point);

  auto goal_future = client->async_send_goal(goal);
  const auto goal_status =
    rclcpp::spin_until_future_complete(node, goal_future, std::chrono::seconds(5));
  if (goal_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Timed out while sending trajectory goal");
    return false;
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(logger, "Trajectory goal was rejected");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto result_status = rclcpp::spin_until_future_complete(
    node,
    result_future,
    std::chrono::duration<double>(duration_sec + server_timeout_sec));
  if (result_status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Timed out waiting for trajectory result");
    return false;
  }

  const auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(logger, "Trajectory failed with result code %d", static_cast<int>(result.code));
    return false;
  }

  RCLCPP_INFO(logger, "Trajectory execution completed");
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("pinocchio_ik");
  const auto logger = node->get_logger();

  const auto robot_description = node->declare_parameter<std::string>("robot_description", "");
  const auto ee_frame = node->declare_parameter<std::string>("ee_frame", "End_Effector");

  // 默认我使用绝对 xyz 目标。比如 x:=0.02 y:=-0.38 z:=0.24。
  const auto x = node->declare_parameter<double>("x", 0.02);
  const auto y = node->declare_parameter<double>("y", -0.3888);
  const auto z = node->declare_parameter<double>("z", 0.2368);

  // use_offset 是给调试留的：true 时继续用 dx/dy/dz 表示从当前末端位置偏移。
  const auto use_offset = node->declare_parameter<bool>("use_offset", false);
  const auto dx = node->declare_parameter<double>("dx", 0.03);
  const auto dy = node->declare_parameter<double>("dy", 0.0);
  const auto dz = node->declare_parameter<double>("dz", 0.0);

  const auto execute = node->declare_parameter<bool>("execute", false);
  const auto trajectory_duration =
    node->declare_parameter<double>("trajectory_duration", 3.0);
  const auto action_name =
    node->declare_parameter<std::string>("action_name", "/arm_controller/follow_joint_trajectory");
  const auto controller_timeout =
    node->declare_parameter<double>("controller_timeout", 5.0);
  const auto joint_state_timeout =
    node->declare_parameter<double>("joint_state_timeout", 2.0);

  soarm100_manipulation::PositionIkOptions ik_options;
  ik_options.max_iterations = node->declare_parameter<int>("max_iterations", 100);
  ik_options.tolerance = node->declare_parameter<double>("tolerance", 1e-4);
  ik_options.damping = node->declare_parameter<double>("damping", 1e-3);
  ik_options.step_size = node->declare_parameter<double>("step_size", 0.4);

  if (robot_description.empty())
  {
    RCLCPP_ERROR(logger, "Parameter robot_description is empty; pass a URDF XML string.");
    rclcpp::shutdown();
    return 1;
  }

  std::unique_ptr<soarm100_manipulation::PinocchioIkSolver> solver;
  try
  {
    solver = std::make_unique<soarm100_manipulation::PinocchioIkSolver>(
      robot_description,
      ee_frame);
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(logger, "Failed to initialize Pinocchio IK solver: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    logger,
    "Pinocchio model: nq=%d nv=%d joints=%zu",
    solver->model().nq,
    solver->model().nv,
    solver->model().names.size());
  RCLCPP_INFO(logger, "End-effector frame: %s", solver->eeFrameName().c_str());

  const auto joint_state = waitForJointState(
    node,
    std::chrono::milliseconds(static_cast<int>(joint_state_timeout * 1000.0)));
  if (!joint_state)
  {
    RCLCPP_WARN(logger, "No /joint_states received; using Pinocchio neutral configuration.");
  }

  // 我把当前 q 作为 IK 起点。初值越接近目标附近，数值 IK 通常越容易收敛。
  Eigen::VectorXd q = solver->makeInitialConfiguration(joint_state ? &(*joint_state) : nullptr);
  const Eigen::Vector3d start_position = solver->endEffectorPosition(q);

  Eigen::Vector3d target_position(x, y, z);
  if (use_offset)
  {
    target_position = start_position + Eigen::Vector3d(dx, dy, dz);
  }

  RCLCPP_INFO(
    logger,
    "Start EE position:  x=%.4f y=%.4f z=%.4f",
    start_position.x(),
    start_position.y(),
    start_position.z());
  RCLCPP_INFO(
    logger,
    "Target EE position: x=%.4f y=%.4f z=%.4f",
    target_position.x(),
    target_position.y(),
    target_position.z());

  const auto result = solver->solvePosition(target_position, q, ik_options);
  if (result.success)
  {
    RCLCPP_INFO(
      logger,
      "IK converged in %d iterations, error=%.6f m",
      result.iterations,
      result.final_error);
  }
  else
  {
    RCLCPP_WARN(logger, "IK did not converge, final error=%.6f m", result.final_error);
  }

  RCLCPP_INFO(logger, "Solved arm joint configuration:");
  const auto arm_positions = solver->extractArmPositions(result.q);
  const auto & arm_joint_names = solver->armJointNames();
  for (std::size_t i = 0; i < arm_joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "  %s = %.6f rad", arm_joint_names[i].c_str(), arm_positions[i]);
  }

  if (result.success && execute)
  {
    const bool executed = sendArmTrajectory(
      node,
      action_name,
      arm_joint_names,
      arm_positions,
      trajectory_duration,
      controller_timeout);
    rclcpp::shutdown();
    return executed ? 0 : 3;
  }

  if (!execute)
  {
    RCLCPP_INFO(logger, "execute=false, so the solved IK target was not sent to a controller.");
  }

  rclcpp::shutdown();
  return result.success ? 0 : 2;
}
