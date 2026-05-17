#ifndef SOARM100_MANIPULATION__ROBOT_INTERFACE_HPP_
#define SOARM100_MANIPULATION__ROBOT_INTERFACE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace soarm100_manipulation
{

class RobotInterface
{
public:
  // 机械臂和夹爪 controller 对外暴露的是 ROS2 action server。
  // 这里用别名缩短类型名，RobotInterface 本身只做 action client。
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;

  // 复用外部 node 创建 action client、订阅 /joint_states、打印日志。
  explicit RobotInterface(const rclcpp::Node::SharedPtr & node);

  // 检查 hardware.launch.py 是否已经启动好 arm/gripper 两个 action server。
  bool waitForServers(double timeout_sec = 5.0);

  // 按固定关节顺序发送关节空间目标：
  // Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll。
  bool moveJoints(const std::vector<double> & positions, double duration_sec = 3.0);

  // 发送 ParallelGripperCommand 到 /gripper_controller/gripper_cmd。
  bool moveGripper(double position, double timeout_sec = 3.0);

  // 常用动作封装，给上层 FSM / demo 使用，避免到处硬编码数值。
  bool openGripper();
  bool closeGripper();
  bool goHome();

  // 返回最近一次 /joint_states 中缓存的状态；还没收到完整状态时返回空/NaN。
  std::vector<double> getCurrentArmPositions() const;
  double getCurrentGripperPosition() const;

  // 暴露关节名，方便 demo、日志、planner 做一致性检查。
  const std::vector<std::string> & armJointNames() const;
  const std::string & gripperJointName() const;

private:
  // /joint_states 回调只更新缓存，不在回调里做耗时控制。
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool waitForArmServer(double timeout_sec);
  bool waitForGripperServer(double timeout_sec);

  // ROS 通信对象：两个 action client + 一个 joint state subscriber。
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // SO-ARM100 的运行时接口配置，必须和 URDF / ros2_control 关节名一致。
  std::vector<std::string> arm_joint_names_;
  std::string gripper_joint_name_;
  std::vector<double> home_positions_;
  double gripper_open_position_;
  double gripper_closed_position_;

  // /joint_states 回调线程和主控制线程都会访问缓存，所以用 mutex 保护。
  mutable std::mutex joint_state_mutex_;
  std::map<std::string, double> latest_joint_positions_;
};

}  // namespace soarm100_manipulation

#endif  // SOARM100_MANIPULATION__ROBOT_INTERFACE_HPP_
