#ifndef SOARM100_MANIPULATION__XBOX_JOINT_TELEOP_HPP_
#define SOARM100_MANIPULATION__XBOX_JOINT_TELEOP_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace soarm100_manipulation
{

class XboxJointTeleop : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;

  XboxJointTeleop();

private:
  void onJointState(const sensor_msgs::msg::JointState & msg);
  double axis(std::size_t index) const;
  bool button(std::size_t index) const;
  double clampJoint(const std::string & joint, double value) const;
  bool readArmPositions(std::vector<double> & positions) const;
  void sendCommand();
  void sendArmGoal(const std::vector<double> & positions);
  void sendGripperCommand();

  const std::vector<std::string> arm_joints_{
    "Shoulder_Rotation",
    "Shoulder_Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
  };
  const std::string gripper_joint_{"Gripper"};

  double axis_deadzone_{0.15};
  double max_joint_velocity_{0.45};
  double max_gripper_velocity_{0.8};
  double command_period_{0.08};
  double trajectory_duration_{0.18};
  double min_goal_delta_{0.01};
  bool log_commands_{false};

  std::unordered_map<std::string, std::pair<double, double>> joint_limits_;
  std::unordered_map<std::string, double> joint_positions_;
  std::optional<std::vector<double>> arm_target_positions_;
  std::optional<sensor_msgs::msg::Joy> latest_joy_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace soarm100_manipulation

#endif  // SOARM100_MANIPULATION__XBOX_JOINT_TELEOP_HPP_
