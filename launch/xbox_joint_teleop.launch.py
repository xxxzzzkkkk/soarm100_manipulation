from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    start_joy = LaunchConfiguration("start_joy")
    start_hardware = LaunchConfiguration("start_hardware")
    serial_port = LaunchConfiguration("serial_port")
    servo_speed = LaunchConfiguration("servo_speed")
    servo_acceleration = LaunchConfiguration("servo_acceleration")
    rviz = LaunchConfiguration("rviz")
    device_id = LaunchConfiguration("device_id")
    deadzone = LaunchConfiguration("deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    axis_deadzone = LaunchConfiguration("axis_deadzone")
    max_joint_velocity = LaunchConfiguration("max_joint_velocity")
    max_gripper_velocity = LaunchConfiguration("max_gripper_velocity")
    command_period = LaunchConfiguration("command_period")
    trajectory_duration = LaunchConfiguration("trajectory_duration")
    min_goal_delta = LaunchConfiguration("min_goal_delta")
    log_commands = LaunchConfiguration("log_commands")

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so_arm_100_bringup"),
                "launch",
                "hardware.launch.py",
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "servo_speed": servo_speed,
            "servo_acceleration": servo_acceleration,
        }.items(),
        condition=IfCondition(start_hardware),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": device_id,
                "deadzone": deadzone,
                "autorepeat_rate": autorepeat_rate,
            }
        ],
        condition=IfCondition(start_joy),
    )

    teleop_node = Node(
        package="soarm100_manipulation",
        executable="xbox_joint_teleop",
        name="xbox_joint_teleop",
        output="screen",
        parameters=[
            {
                "axis_deadzone": axis_deadzone,
                "max_joint_velocity": max_joint_velocity,
                "max_gripper_velocity": max_gripper_velocity,
                "command_period": command_period,
                "trajectory_duration": trajectory_duration,
                "min_goal_delta": min_goal_delta,
                "log_commands": log_commands,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("so_arm_100_description"),
                "rviz",
                "so_arm_100.rviz",
            ),
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_joy",
                default_value="true",
                description="Start joy_node in this launch file.",
            ),
            DeclareLaunchArgument(
                "start_hardware",
                default_value="true",
                description="Start SO-ARM100 hardware launch in this launch file.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Servo controller board serial port.",
            ),
            DeclareLaunchArgument(
                "servo_speed",
                default_value="2400",
                description="Servo move speed in ticks/s.",
            ),
            DeclareLaunchArgument(
                "servo_acceleration",
                default_value="50",
                description="Servo acceleration in ticks/s^2.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz to visualize /joint_states and TF.",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Joystick device id used by joy_node.",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.05",
                description="joy_node axis deadzone.",
            ),
            DeclareLaunchArgument(
                "autorepeat_rate",
                default_value="40.0",
                description="joy_node repeat rate in Hz.",
            ),
            DeclareLaunchArgument(
                "axis_deadzone",
                default_value="0.15",
                description="Teleop axis deadzone.",
            ),
            DeclareLaunchArgument(
                "max_joint_velocity",
                default_value="0.45",
                description="Maximum arm joint jogging velocity in rad/s.",
            ),
            DeclareLaunchArgument(
                "max_gripper_velocity",
                default_value="0.40",
                description="Maximum gripper jogging velocity in rad/s.",
            ),
            DeclareLaunchArgument(
                "command_period",
                default_value="0.06",
                description="Teleop command period in seconds.",
            ),
            DeclareLaunchArgument(
                "trajectory_duration",
                default_value="0.12",
                description="Duration of each small trajectory command.",
            ),
            DeclareLaunchArgument(
                "min_goal_delta",
                default_value="0.001",
                description="Minimum joint target change before sending a new action goal.",
            ),
            DeclareLaunchArgument(
                "log_commands",
                default_value="true",
                description="Print throttled arm target commands.",
            ),
            hardware_launch,
            joy_node,
            teleop_node,
            rviz_node,
        ]
    )
