#ifndef SOARM100_MANIPULATION__PINOCCHIO_IK_SOLVER_HPP_
#define SOARM100_MANIPULATION__PINOCCHIO_IK_SOLVER_HPP_

#include <string>
#include <vector>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace soarm100_manipulation
{

struct PositionIkOptions
{
  int max_iterations{100};
  double tolerance{1e-4};
  double damping{1e-3};
  double step_size{0.4};
};

struct PositionIkResult
{
  bool success{false};
  int iterations{0};
  double final_error{0.0};
  Eigen::VectorXd q;
};

class PinocchioIkSolver
{
public:
  PinocchioIkSolver(const std::string & robot_description, const std::string & ee_frame);

  Eigen::VectorXd makeInitialConfiguration(
    const sensor_msgs::msg::JointState * joint_state = nullptr) const;

  Eigen::Vector3d endEffectorPosition(const Eigen::VectorXd & q);

  PositionIkResult solvePosition(
    const Eigen::Vector3d & target_position,
    const Eigen::VectorXd & initial_q,
    const PositionIkOptions & options);

  std::vector<double> extractArmPositions(const Eigen::VectorXd & q) const;

  const std::vector<std::string> & armJointNames() const;
  const pinocchio::Model & model() const;
  const std::string & eeFrameName() const;

private:
  void clampToPositionLimits(Eigen::VectorXd & q) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_;
  std::string ee_frame_name_;
  std::vector<std::string> arm_joint_names_;
};

}  // namespace soarm100_manipulation

#endif  // SOARM100_MANIPULATION__PINOCCHIO_IK_SOLVER_HPP_
