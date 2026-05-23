#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "soarm100_manipulation/xbox_joint_teleop.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<soarm100_manipulation::XboxJointTeleop>());
  rclcpp::shutdown();
  return 0;
}
