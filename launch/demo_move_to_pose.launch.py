from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    moveit_launch_dir = os.path.join(
        get_package_share_directory("so_arm_100_moveit_config"),
        "launch",
    )

    dx = LaunchConfiguration("dx")
    dy = LaunchConfiguration("dy")
    dz = LaunchConfiguration("dz")
    planning_group = LaunchConfiguration("planning_group")
    start_move_group = LaunchConfiguration("start_move_group")
    rviz = LaunchConfiguration("rviz")
    demo_delay = LaunchConfiguration("demo_delay")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, "move_group.launch.py")
        ),
        condition=IfCondition(start_move_group),
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, "moveit_rviz.launch.py")
        ),
        condition=IfCondition(rviz),
    )

    demo_node = Node(
        package="soarm100_manipulation",
        executable="demo_move_to_pose",
        name="demo_move_to_pose",
        output="screen",
        parameters=[
            {
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "planning_group": planning_group,
            }
        ],
    )

    delayed_demo_with_move_group = TimerAction(
        period=demo_delay,
        actions=[demo_node],
        condition=IfCondition(start_move_group),
    )

    demo_without_move_group_delay = Node(
        package="soarm100_manipulation",
        executable="demo_move_to_pose",
        name="demo_move_to_pose",
        output="screen",
        parameters=[
            {
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "planning_group": planning_group,
            }
        ],
        condition=UnlessCondition(start_move_group),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dx",
                default_value="0.03",
                description="End-effector x offset in meters.",
            ),
            DeclareLaunchArgument(
                "dy",
                default_value="0.0",
                description="End-effector y offset in meters.",
            ),
            DeclareLaunchArgument(
                "dz",
                default_value="0.0",
                description="End-effector z offset in meters.",
            ),
            DeclareLaunchArgument(
                "planning_group",
                default_value="arm",
                description="MoveIt planning group.",
            ),
            DeclareLaunchArgument(
                "start_move_group",
                default_value="true",
                description="Start so_arm_100_moveit_config move_group.launch.py.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start MoveIt RViz for visualization.",
            ),
            DeclareLaunchArgument(
                "demo_delay",
                default_value="4.0",
                description="Delay before running the demo when move_group is started here.",
            ),
            move_group,
            moveit_rviz,
            delayed_demo_with_move_group,
            demo_without_move_group_delay,
        ]
    )
