cmake_minimum_required(VERSION 3.8)
project(soarm100_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
  pinocchio
)

add_library(robot_interface
  src/robot_interface.cpp
)
target_include_directories(robot_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_library(pinocchio_ik_solver
  src/pinocchio_ik_solver.cpp
)
target_include_directories(pinocchio_ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pinocchio_ik_solver
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(pinocchio_ik_solver
  Eigen3::Eigen
)

add_library(xbox_joint_teleop_lib
  src/xbox_joint_teleop.cpp
)
target_include_directories(xbox_joint_teleop_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xbox_joint_teleop_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(pinocchio_ik
  src/pinocchio_ik.cpp
)
target_include_directories(pinocchio_ik PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(pinocchio_ik
  pinocchio_ik_solver
)
ament_target_dependencies(pinocchio_ik
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(xbox_joint_teleop
  src/xbox_joint_teleop_node.cpp
)
target_link_libraries(xbox_joint_teleop
  xbox_joint_teleop_lib
)
ament_target_dependencies(xbox_joint_teleop
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS robot_interface pinocchio_ik_solver xbox_joint_teleop_lib
  EXPORT export_robot_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS pinocchio_ik xbox_joint_teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_robot_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
