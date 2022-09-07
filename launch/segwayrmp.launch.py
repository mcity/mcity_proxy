import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()
    package_name = "mcity_proxy"
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # * Segway RMP *
    ld.add_action(
        Node(
            package="segwayrmp",
            executable="SmartCar",
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/set_chassis_enable ",
                    "segway_msgs/srv/RosSetChassisEnableCmd ",
                    '"{ros_set_chassis_enable_cmd: True}"',
                ]
            ],
            shell=True,
        )
    )

    return ld
