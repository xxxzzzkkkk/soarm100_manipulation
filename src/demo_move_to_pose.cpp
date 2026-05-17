#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "soarm100_manipulation/robot_interface.hpp"

namespace
{

void printPositions(
  const rclcpp::Logger & logger,
  const std::string & label,
  const std::vector<std::string> & names,
  const std::vector<double> & positions)
{
  if (positions.empty())
  {
    RCLCPP_WARN(logger, "%s: joint state is not available yet", label.c_str());
    return;
  }

  std::string text = label + ":";
  for (size_t i = 0; i < names.size() && i < positions.size(); ++i)
  {
    text += " " + names[i] + "=" + std::to_string(positions[i]);
  }
  RCLCPP_INFO(logger, "%s", text.c_str());
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("demo_move_to_pose");
  soarm100_manipulation::RobotInterface robot(node);
  const auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Waiting for SO-ARM100 action servers...");
  if (!robot.waitForServers(10.0))
  {
    RCLCPP_ERROR(logger, "RobotInterface is not ready. Is hardware.launch.py running?");
    rclcpp::shutdown();
    return 1;
  }

  // Give /joint_states a short moment to arrive before printing the initial state.
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  rclcpp::spin_some(node);

  printPositions(
    logger, "Initial arm state", robot.armJointNames(), robot.getCurrentArmPositions());
  RCLCPP_INFO(logger, "Initial gripper state: %.3f", robot.getCurrentGripperPosition());

  RCLCPP_INFO(logger, "Running RobotInterface smoke test sequence");

  bool ok = true;
  ok = robot.goHome() && ok;
  ok = robot.openGripper() && ok;

  // Conservative joint-space target for validating the action/control chain.
  ok = robot.moveJoints({0.0, -0.30, 0.50, 0.20, 0.0}, 3.0) && ok;
  ok = robot.closeGripper() && ok;

  // Always try to return to home even if an earlier command failed.
  ok = robot.goHome() && ok;

  rclcpp::spin_some(node);
  printPositions(logger, "Final arm state", robot.armJointNames(), robot.getCurrentArmPositions());
  RCLCPP_INFO(logger, "Final gripper state: %.3f", robot.getCurrentGripperPosition());

  if (!ok)
  {
    RCLCPP_ERROR(logger, "Demo failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Demo completed");
  rclcpp::shutdown();
  return 0;
}
