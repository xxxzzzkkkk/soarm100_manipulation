#include "soarm100_manipulation/pinocchio_ik_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace soarm100_manipulation
{

PinocchioIkSolver::PinocchioIkSolver(
  const std::string & robot_description,
  const std::string & ee_frame)
: model_(),
  data_(model_),
  ee_frame_id_(0),
  ee_frame_name_(ee_frame),
  arm_joint_names_({
    "Shoulder_Rotation",
    "Shoulder_Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
  })
{
  // 我在构造函数里把 URDF XML 解析成 Pinocchio Model。
  // Model 保存机器人结构，后面的 FK/Jacobian/IK 都围绕它做。
  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);

  if (!model_.existFrame(ee_frame_name_))
  {
    throw std::runtime_error("Pinocchio model does not contain frame `" + ee_frame_name_ + "`");
  }
  ee_frame_id_ = model_.getFrameId(ee_frame_name_);

  for (const auto & joint_name : arm_joint_names_)
  {
    if (!model_.existJointName(joint_name))
    {
      throw std::runtime_error("Pinocchio model does not contain joint `" + joint_name + "`");
    }
  }
}

Eigen::VectorXd PinocchioIkSolver::makeInitialConfiguration(
  const sensor_msgs::msg::JointState * joint_state) const
{
  // 我把 q 当成 Pinocchio 的“广义位置向量”。
  // 对这台机械臂来说，它基本就是各个 revolute joint 的角度。
  Eigen::VectorXd q = pinocchio::neutral(model_);
  if (joint_state == nullptr)
  {
    return q;
  }

  // 我先用名字建索引，避免依赖 /joint_states 里的数组顺序。
  std::unordered_map<std::string, double> positions;
  for (std::size_t i = 0; i < joint_state->name.size() && i < joint_state->position.size(); ++i)
  {
    positions[joint_state->name[i]] = joint_state->position[i];
  }

  // model.joints[0] 是 universe，不是真实关节，所以我从 1 开始。
  for (pinocchio::JointIndex joint_id = 1; joint_id < model_.joints.size(); ++joint_id)
  {
    const auto it = positions.find(model_.names[joint_id]);
    if (it == positions.end())
    {
      continue;
    }

    // idx_q 是这个关节在 q 向量里的下标。SO-ARM100 的关节都是 1DoF。
    const int idx_q = model_.joints[joint_id].idx_q();
    if (idx_q >= 0 && model_.joints[joint_id].nq() == 1)
    {
      q[idx_q] = it->second;
    }
  }

  return q;
}

Eigen::Vector3d PinocchioIkSolver::endEffectorPosition(const Eigen::VectorXd & q)
{
  // 我先更新 joint 位姿，再更新 frame 位姿，然后读取末端 frame 的 xyz。
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[ee_frame_id_].translation();
}

PositionIkResult PinocchioIkSolver::solvePosition(
  const Eigen::Vector3d & target_position,
  const Eigen::VectorXd & initial_q,
  const PositionIkOptions & options)
{
  PositionIkResult result;
  result.q = initial_q;

  // 我这里只做位置 IK，不硬解姿态。
  // SO-ARM100 手臂本体是 5DoF，完整 6D 位姿任务通常会过约束。
  for (int iter = 0; iter < options.max_iterations; ++iter)
  {
    pinocchio::forwardKinematics(model_, data_, result.q);
    pinocchio::updateFramePlacements(model_, data_);

    const Eigen::Vector3d current_position = data_.oMf[ee_frame_id_].translation();
    const Eigen::Vector3d error = target_position - current_position;
    result.iterations = iter;
    result.final_error = error.norm();

    if (result.final_error < options.tolerance)
    {
      result.success = true;
      return result;
    }

    // full_jacobian 是 6 x nv，前三行是线速度，后三行是角速度。
    // 我只解 xyz，所以只拿前三行。
    pinocchio::computeJointJacobians(model_, data_, result.q);
    Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(6, model_.nv);
    full_jacobian.setZero();
    pinocchio::getFrameJacobian(
      model_,
      data_,
      ee_frame_id_,
      pinocchio::LOCAL_WORLD_ALIGNED,
      full_jacobian);

    const Eigen::MatrixXd linear_jacobian = full_jacobian.topRows(3);

    // 我用阻尼最小二乘，避免接近奇异位形时 dq 变得特别大。
    const Eigen::Matrix3d task_matrix =
      linear_jacobian * linear_jacobian.transpose() +
      options.damping * options.damping * Eigen::Matrix3d::Identity();
    const Eigen::VectorXd dq = linear_jacobian.transpose() * task_matrix.ldlt().solve(error);

    result.q = pinocchio::integrate(model_, result.q, options.step_size * dq);
    clampToPositionLimits(result.q);
  }

  result.final_error = (target_position - endEffectorPosition(result.q)).norm();
  return result;
}

std::vector<double> PinocchioIkSolver::extractArmPositions(const Eigen::VectorXd & q) const
{
  std::vector<double> positions;
  positions.reserve(arm_joint_names_.size());

  for (const auto & joint_name : arm_joint_names_)
  {
    const auto joint_id = model_.getJointId(joint_name);
    const int idx_q = model_.joints[joint_id].idx_q();
    if (idx_q < 0 || model_.joints[joint_id].nq() != 1)
    {
      throw std::runtime_error("Joint `" + joint_name + "` is not a 1-DoF joint");
    }
    positions.push_back(q[idx_q]);
  }

  return positions;
}

const std::vector<std::string> & PinocchioIkSolver::armJointNames() const
{
  return arm_joint_names_;
}

const pinocchio::Model & PinocchioIkSolver::model() const
{
  return model_;
}

const std::string & PinocchioIkSolver::eeFrameName() const
{
  return ee_frame_name_;
}

void PinocchioIkSolver::clampToPositionLimits(Eigen::VectorXd & q) const
{
  // 我每次更新 q 后都做限位裁剪，先保证不会明显跑出 URDF limit。
  for (int i = 0; i < q.size(); ++i)
  {
    const double lower = model_.lowerPositionLimit[i];
    const double upper = model_.upperPositionLimit[i];
    if (std::isfinite(lower) && std::isfinite(upper) && lower < upper)
    {
      q[i] = std::clamp(q[i], lower, upper);
    }
  }
}

}  // namespace soarm100_manipulation
