from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    use_offset = LaunchConfiguration("use_offset")
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
                "x": x,
                "y": y,
                "z": z,
                "use_offset": use_offset,
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
                "x",
                default_value="0.02",
                description="Absolute target end-effector x position in meters.",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="-0.3888",
                description="Absolute target end-effector y position in meters.",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.2368",
                description="Absolute target end-effector z position in meters.",
            ),
            DeclareLaunchArgument(
                "use_offset",
                default_value="false",
                description="Use dx/dy/dz as an offset from the current end-effector position.",
            ),
            DeclareLaunchArgument(
                "dx",
                default_value="0.03",
                description="End-effector x offset in meters when use_offset=true.",
            ),
            DeclareLaunchArgument(
                "dy",
                default_value="0.0",
                description="End-effector y offset in meters when use_offset=true.",
            ),
            DeclareLaunchArgument(
                "dz",
                default_value="0.0",
                description="End-effector z offset in meters when use_offset=true.",
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
