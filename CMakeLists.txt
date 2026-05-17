cmake_minimum_required(VERSION 3.8)
project(soarm100_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pinocchio REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
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

add_library(motion_planner
  src/motion_planner.cpp
)
target_include_directories(motion_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motion_planner
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(demo_move_to_pose
  src/demo_move_to_pose.cpp
)
target_link_libraries(demo_move_to_pose
  motion_planner
)
ament_target_dependencies(demo_move_to_pose
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(pinocchio_ik
  src/pinocchio_ik.cpp
)
ament_target_dependencies(pinocchio_ik
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
  TARGETS robot_interface motion_planner
  EXPORT export_robot_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS demo_move_to_pose pinocchio_ik
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
