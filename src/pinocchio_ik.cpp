#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

const std::vector<std::string> kArmJointNames = {
  "Shoulder_Rotation",
  "Shoulder_Pitch",
  "Elbow",
  "Wrist_Pitch",
  "Wrist_Roll",
};

builtin_interfaces::msg::Duration secondsToDurationMsg(double seconds)
{
  builtin_interfaces::msg::Duration msg;
  msg.sec = static_cast<int32_t>(std::floor(seconds));
  msg.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(msg.sec)) * 1e9);
  return msg;
}

// 我先等一帧 /joint_states。
//
// IK 需要一个初始关节角 q。我最希望用机械臂当前真实/仿真的关节角，
// 因为数值 IK 从当前状态附近开始迭代，一般更容易收敛。
// 如果我等不到 /joint_states，后面就退回到 Pinocchio 的 neutral configuration。
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

Eigen::VectorXd makeInitialConfiguration(
  const pinocchio::Model & model,
  const std::optional<sensor_msgs::msg::JointState> & joint_state)
{
  // 我把 q 当成“广义位置向量”，也就是 Pinocchio 表示机器人姿态的一整串数字。
  // 对 SO-ARM100 这种全 revolute 机械臂来说，可以简单理解成：
  //   q = [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Gripper]
  //
  // 更复杂的机器人可能有 floating base、四元数关节等，所以 Pinocchio
  // 不建议我手写 q 的长度和排列，而是从 model 里读取。
  Eigen::VectorXd q = pinocchio::neutral(model);
  if (!joint_state)
  {
    return q;
  }

  // /joint_states 里是 name[] 和 position[] 两个数组。
  // 我先转成 map，方便按关节名字查角度，避免依赖 ROS 消息里的顺序。
  std::unordered_map<std::string, double> positions;
  for (std::size_t i = 0; i < joint_state->name.size() && i < joint_state->position.size(); ++i)
  {
    positions[joint_state->name[i]] = joint_state->position[i];
  }

  // model.joints[0] 是 universe，不是真实关节，所以我从 1 开始。
  for (pinocchio::JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
  {
    const auto it = positions.find(model.names[joint_id]);
    if (it == positions.end())
    {
      continue;
    }

    // 我用 idx_q 找这个关节在 q 向量里的起始下标。
    // nq()==1 表示这个关节只占 q 里的一个数；revolute/prismatic 都是这样。
    const int idx_q = model.joints[joint_id].idx_q();
    if (idx_q >= 0 && model.joints[joint_id].nq() == 1)
    {
      q[idx_q] = it->second;
    }
  }

  return q;
}

std::vector<double> extractArmPositions(const pinocchio::Model & model, const Eigen::VectorXd & q)
{
  std::vector<double> positions;
  positions.reserve(kArmJointNames.size());

  for (const auto & joint_name : kArmJointNames)
  {
    if (!model.existJointName(joint_name))
    {
      throw std::runtime_error("Pinocchio model does not contain joint `" + joint_name + "`");
    }

    const auto joint_id = model.getJointId(joint_name);
    const int idx_q = model.joints[joint_id].idx_q();
    if (idx_q < 0 || model.joints[joint_id].nq() != 1)
    {
      throw std::runtime_error("Joint `" + joint_name + "` is not a 1-DoF joint");
    }
    positions.push_back(q[idx_q]);
  }

  return positions;
}

void clampToPositionLimits(const pinocchio::Model & model, Eigen::VectorXd & q)
{
  // 数值 IK 每一步都会更新 q，可能会把关节推到 URDF limit 外。
  // 我这里先做一个最朴素的限位裁剪，保证结果不会明显越界。
  for (int i = 0; i < q.size(); ++i)
  {
    const double lower = model.lowerPositionLimit[i];
    const double upper = model.upperPositionLimit[i];
    if (std::isfinite(lower) && std::isfinite(upper) && lower < upper)
    {
      q[i] = std::clamp(q[i], lower, upper);
    }
  }
}

bool solvePositionIk(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const pinocchio::FrameIndex ee_frame_id,
  const Eigen::Vector3d & target_position,
  Eigen::VectorXd & q,
  const int max_iterations,
  const double tolerance,
  const double damping,
  const double step_size,
  rclcpp::Logger logger)
{
  // 我这里写的是“只跟踪末端位置”的迭代 IK：
  //
  // 1. 我用当前 q 做 FK，得到末端当前位置 x(q)
  // 2. 我算位置误差 e = target - x(q)
  // 3. 我用 Jacobian 建立近似关系：dx = J * dq
  // 4. 我反过来求一个 dq，让末端往误差变小的方向走
  // 5. 我更新 q，然后重复
  //
  // 我不在这里硬解姿态，因为 SO-ARM100 手臂本体只有 5 DoF。
  // 末端完整位姿是 6D 任务，普通情况下会过约束；这个学习版先把位置 IK 做稳。
  for (int iter = 0; iter < max_iterations; ++iter)
  {
    // 我先调用 forwardKinematics，让 Pinocchio 更新每个 joint 的位姿。
    // 再调用 updateFramePlacements，让每个 frame 的位姿也刷新。
    // URDF 里的 link/joint/末端工具点，我通常都以 frame 的形式取出来。
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // 我从 data.oMf[frame_id] 里读 frame 相对于 world/origin(o) 的 SE3 位姿。
    // 我只取 translation()，所以这个 IK 只关心末端 xyz，不硬约束姿态。
    const Eigen::Vector3d current_position = data.oMf[ee_frame_id].translation();
    const Eigen::Vector3d error = target_position - current_position;
    if (error.norm() < tolerance)
    {
      RCLCPP_INFO(logger, "IK converged in %d iterations, error=%.6f m", iter, error.norm());
      return true;
    }

    // 我用 Jacobian 描述“关节速度 -> 末端空间速度”的线性近似关系。
    // full_jacobian 是 6 x nv：
    //   前 3 行：线速度部分 vx, vy, vz
    //   后 3 行：角速度部分 wx, wy, wz
    pinocchio::computeJointJacobians(model, data, q);
    Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(6, model.nv);
    full_jacobian.setZero();
    pinocchio::getFrameJacobian(
      model,
      data,
      ee_frame_id,
      pinocchio::LOCAL_WORLD_ALIGNED,
      full_jacobian);

    // 我只解位置 IK，所以只使用 Jacobian 的前三行，也就是线速度部分。
    const Eigen::MatrixXd linear_jacobian = full_jacobian.topRows(3);

    // 我这里用阻尼最小二乘 DLS：
    //
    // 如果用普通伪逆，可以写成：
    //   dq = J^T * (J * J^T)^-1 * error
    //
    // 但靠近奇异位形时，J * J^T 可能很难求逆，dq 会突然变得很大。
    // 所以我加一个 damping^2 * I：
    //   dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
    //
    // 我把 lambda 调大一点会更稳，但动作更保守；调小一点更灵敏，但更怕奇异点。
    const Eigen::Matrix3d task_matrix =
      linear_jacobian * linear_jacobian.transpose() +
      damping * damping * Eigen::Matrix3d::Identity();
    const Eigen::VectorXd dq = linear_jacobian.transpose() * task_matrix.ldlt().solve(error);

    // 我用 integrate，而不是直接 q += dq，因为 integrate 更通用。
    // 对普通 revolute 关节效果接近相加；对四元数/floating base 这类配置，
    // integrate 会用正确的流形方式更新。
    //
    // 我用 step_size 控制步长，防止每次走得太猛导致震荡。
    q = pinocchio::integrate(model, q, step_size * dq);
    clampToPositionLimits(model, q);
  }

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  const double final_error = (target_position - data.oMf[ee_frame_id].translation()).norm();
  RCLCPP_WARN(logger, "IK did not converge, final error=%.6f m", final_error);
  return false;
}

bool sendArmTrajectory(
  const rclcpp::Node::SharedPtr & node,
  const std::string & action_name,
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
  goal.trajectory.joint_names = kArmJointNames;

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
  const auto max_iterations = node->declare_parameter<int>("max_iterations", 100);
  const auto tolerance = node->declare_parameter<double>("tolerance", 1e-4);
  const auto damping = node->declare_parameter<double>("damping", 1e-3);
  const auto step_size = node->declare_parameter<double>("step_size", 0.4);

  if (robot_description.empty())
  {
    RCLCPP_ERROR(logger, "Parameter robot_description is empty; pass a URDF XML string.");
    rclcpp::shutdown();
    return 1;
  }

  // 我把 Model 当作机器人“不随 q 改变”的结构信息：
  // 关节树、关节名字、limit、每个关节的自由度、frame 列表等。
  //
  // 我把 Data 当作计算缓存：
  // FK/Jacobian/动力学计算出来的中间结果都会放在这里。
  // 常见写法就是：Model 创建一次，Data 跟着 Model 创建，然后反复复用。
  pinocchio::Model model;
  try
  {
    // launch 文件把 xacro 展开的 URDF XML 字符串放进 robot_description。
    // 我让 Pinocchio 从 URDF 里解析出运动学模型。
    pinocchio::urdf::buildModelFromXML(robot_description, model);
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(logger, "Failed to build Pinocchio model from URDF: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  pinocchio::Data data(model);

  // ee_frame 是我要控制的末端坐标系。
  // 当前 launch 默认用 URDF 里的 End_Effector frame。
  if (!model.existFrame(ee_frame))
  {
    RCLCPP_ERROR(logger, "Frame `%s` was not found in the Pinocchio model.", ee_frame.c_str());
    RCLCPP_INFO(logger, "Available frames:");
    for (const auto & frame : model.frames)
    {
      RCLCPP_INFO(logger, "  %s", frame.name.c_str());
    }
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Pinocchio model: nq=%d nv=%d joints=%zu", model.nq, model.nv, model.names.size());
  RCLCPP_INFO(logger, "End-effector frame: %s", ee_frame.c_str());

  const auto joint_state = waitForJointState(
    node,
    std::chrono::milliseconds(static_cast<int>(joint_state_timeout * 1000.0)));
  if (!joint_state)
  {
    RCLCPP_WARN(logger, "No /joint_states received; using Pinocchio neutral configuration.");
  }

  // 我把 q 作为 IK 的起点。初值越接近目标附近，数值 IK 通常越容易收敛。
  Eigen::VectorXd q = makeInitialConfiguration(model, joint_state);

  // 我先做一次 FK，拿到当前末端位置。
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  const auto ee_frame_id = model.getFrameId(ee_frame);
  const Eigen::Vector3d start_position = data.oMf[ee_frame_id].translation();

  // 我这个例子不直接输入绝对目标点，而是从当前位置偏移 dx/dy/dz。
  // 例如 dx:=0.03 就是让末端在 world x 方向移动 3cm。
  const Eigen::Vector3d target_position = start_position + Eigen::Vector3d(dx, dy, dz);

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
  const bool ok = solvePositionIk(
    model,
    data,
    ee_frame_id,
    target_position,
    q,
    max_iterations,
    tolerance,
    damping,
    step_size,
    logger);

  RCLCPP_INFO(logger, "Solved joint configuration:");
  for (pinocchio::JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
  {
    const int idx_q = model.joints[joint_id].idx_q();
    if (idx_q >= 0 && model.joints[joint_id].nq() == 1)
    {
      RCLCPP_INFO(logger, "  %s = %.6f rad", model.names[joint_id].c_str(), q[idx_q]);
    }
  }

  if (ok && execute)
  {
    const auto arm_positions = extractArmPositions(model, q);
    const bool executed = sendArmTrajectory(
      node,
      action_name,
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
  return ok ? 0 : 2;
}
