import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()
    package_name = "mcity_proxy"
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # * RVIZ *
    rviz_config_file_path = "rviz/nav2_config.rviz"
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    # RVIZ Toggle
    rviz_toggle = LaunchConfiguration("rviz_toggle")
    rviz_toggle_arg = DeclareLaunchArgument(
        name="rviz_toggle",
        default_value="True",
        description="Flag to enable rviz",
    )
    ld.add_action(rviz_toggle_arg)
    # RVIZ Config File
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=default_rviz_config_path,
        description="Full path to the RVIZ config file to use",
    )
    ld.add_action(rviz_config_file_arg)
    # Launch RViz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(rviz_toggle),
        )
    )

    # * STEERING *
    steering_toggle = LaunchConfiguration("steering_toggle")
    steering_toggle_arg = DeclareLaunchArgument(
        name="steering_toggle",
        default_value="True",
        description="Determines whether or not to start the rqt_robot_steering node.",
    )
    ld.add_action(steering_toggle_arg)
    ld.add_action(
        Node(
            package="rqt_robot_steering",
            executable="rqt_robot_steering",
            condition=IfCondition(steering_toggle),
        )
    )

    ntrip_launch_dir = FindPackageShare(package="ntrip_client").find("ntrip_client")
    ntrip_toggle = LaunchConfiguration("ntrip_toggle")
    ntrip_toggle_arg = DeclareLaunchArgument(
        name="ntrip_toggle",
        default_value="True",
        description="Determines whether or not to start the NTRIP Client for RTCM correction data.",
    )
    ld.add_action(ntrip_toggle_arg)
    ntrip_host = LaunchConfiguration("ntrip_host")
    ntrip_host_arg = DeclareLaunchArgument(
        name="ntrip_host",
        default_value="141.211.25.177",
        description="Set hostname/ip address of NTRIP server",
    )
    ld.add_action(ntrip_host_arg)
    ntrip_port = LaunchConfiguration("ntrip_port")
    ntrip_port_arg = DeclareLaunchArgument(
        name="ntrip_port",
        default_value="2102",
        description="Set port number of NTRIP server",
    )
    ld.add_action(ntrip_port_arg)
    ntrip_mountpoint = LaunchConfiguration("ntrip_mountpoint")
    ntrip_mountpoint_arg = DeclareLaunchArgument(
        name="ntrip_mountpoint",
        default_value="MTF",
        description="Set mountpoint of NTRIP server",
    )
    ld.add_action(ntrip_mountpoint_arg)
    ntrip_username = LaunchConfiguration("ntrip_username")
    ntrip_username_arg = DeclareLaunchArgument(
        name="ntrip_username",
        description="Set username of NTRIP server",
        default_value=os.getenv("NTRIP_USERNAME"),
    )
    ld.add_action(ntrip_username_arg)
    ntrip_password = LaunchConfiguration("ntrip_password")
    ntrip_password_arg = DeclareLaunchArgument(
        name="ntrip_password",
        description="Set password of NTRIP server",
        default_value=os.getenv("NTRIP_PASSWORD"),
    )
    ld.add_action(ntrip_password_arg)
    # Launch NTRIP Client
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ntrip_launch_dir, "ntrip_client_launch.py")
            ),
            launch_arguments={
                "host": ntrip_host,
                "port": ntrip_port,
                "mountpoint": ntrip_mountpoint,
                "username": ntrip_username,
                "password": ntrip_password,
            }.items(),
            condition=IfCondition(ntrip_toggle),
        )
    )

    # Segway Enable Service Call
    segway_enable = LaunchConfiguration("segway_enable")
    segway_enable_arg = DeclareLaunchArgument(
        name="segway_enable",
        default_value="False",
        description="Determines whether or not to enable the Segway RMP."
    )
    ld.add_action(segway_enable_arg)
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
            condition=IfCondition(segway_enable),
        )
    )
