"""Launch FAST-LOCALIZATION's standard PointCloud2 RobotDog profile on ROS 2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("fast_localization"), "config", "robotdog.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_dir",
            default_value="",
            description="Required FAST-LIVO2 run directory containing keyframes/.",
        ),
        DeclareLaunchArgument(
            "trajectory_dir",
            default_value="",
            description="Optional directory for automatic CSV and PCD trajectory export.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Publish camera_init -> body TF. Keep false during isolated testing.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock when replaying a bag with --clock.",
        ),
        Node(
            package="fast_localization",
            executable="fast_localization",
            name="fast_localization",
            output="screen",
            parameters=[
                config_file,
                {
                    "map.directory": LaunchConfiguration("map_dir"),
                    "trajectory.output_directory": LaunchConfiguration("trajectory_dir"),
                    "publish.tf": LaunchConfiguration("publish_tf"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
    ])
