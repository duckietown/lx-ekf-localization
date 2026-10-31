import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    veh = LaunchConfiguration("veh")
    share = get_package_share_directory("ekf_localization")
    config = os.path.join(share, "config", "ekf_localization_node", "default.yaml")
    map_file = os.path.join(share, "map.yaml")
    encoder_pose_launch = os.path.join(get_package_share_directory("encoder_pose"), "launch",
                                       "encoder_pose_node.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("veh", default_value=EnvironmentVariable("VEHICLE_NAME"),
                              description="robot name (namespace of every topic)"),
        Node(
            package="ekf_localization",
            executable="ekf_localization_node",
            name="ekf_localization_node",
            namespace=veh,
            parameters=[config, {"map_file": map_file}],
            output="screen",
            emulate_tty=True,
        ),
        # dead-reckoning reference estimate (recipe-provided node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(encoder_pose_launch),
            launch_arguments={"veh": veh}.items(),
        ),
    ])
