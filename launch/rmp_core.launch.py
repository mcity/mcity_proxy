from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="segwayrmp", executable="SmartCar", name="rmp_base_controller"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ublox_gps"),
                                "launch",
                                "ublox_gps_node-launch.py",
                            ]
                        )
                    ]
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="GPS_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0", "0", "base_link", "gps"],
            ),
        ]
    )
