from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dx = LaunchConfiguration("dx")
    dy = LaunchConfiguration("dy")
    dz = LaunchConfiguration("dz")
    ee_frame = LaunchConfiguration("ee_frame")
    execute = LaunchConfiguration("execute")
    trajectory_duration = LaunchConfiguration("trajectory_duration")

    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("so_arm_100_description"),
                    "urdf",
                    "so_arm_100_5dof.urdf.xacro",
                ]
            ),
            " use_fake_hardware:=true",
        ]
    )

    ik_node = Node(
        package="soarm100_manipulation",
        executable="pinocchio_ik",
        name="pinocchio_ik",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "ee_frame": ee_frame,
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "execute": execute,
                "trajectory_duration": trajectory_duration,
            }
        ],
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
                "ee_frame",
                default_value="End_Effector",
                description="Pinocchio frame used as the IK target.",
            ),
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Send the solved joint target to /arm_controller when true.",
            ),
            DeclareLaunchArgument(
                "trajectory_duration",
                default_value="3.0",
                description="Seconds for the joint trajectory controller target point.",
            ),
            ik_node,
        ]
    )
