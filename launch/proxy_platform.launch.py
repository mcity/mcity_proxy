import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
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

    # * Mcity Autonomous Proxy Functionality *
    mcity_proxy_toggle = LaunchConfiguration("mcity_proxy_toggle")
    mcity_proxy_toggle_arg = DeclareLaunchArgument(
        name="mcity_proxy_toggle",
        default_value="True",
        description="Determines whether or not to start the Mcity Autonomous Proxy Functionality.",
    )
    ld.add_action(mcity_proxy_toggle_arg)
    # Launch Proxy Functionality
    ld.add_action(
        Node(
            package="mcity_proxy",
            executable="cmd_vel_spin",
            condition=IfCondition(mcity_proxy_toggle),
            parameters=[{"timeout_period": 0.1}],
        )
    )
    ld.add_action(
        Node(
            package="mcity_proxy",
            executable="move_distance_action_server",
            condition=IfCondition(mcity_proxy_toggle),
        )
    )
    ld.add_action(
        Node(
            package="mcity_proxy",
            executable="action_manager",
            condition=IfCondition(mcity_proxy_toggle),
        )
    )
    ld.add_action(
        Node(
            package="mcity_proxy",
            executable="linear_velocity_open_loop_controller",
            condition=IfCondition(mcity_proxy_toggle),
        )
    )

    # * ROS Bridge *
    ros_bridge_dir = FindPackageShare(package="rosbridge_server").find("rosbridge_server")
    ros_bridge_launch_dir = os.path.join(ros_bridge_dir, "launch")
    ld.add_action(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(ros_bridge_launch_dir, "rosbridge_websocket_launch.xml")
            )
        )
    )

    # * UBLOX RTK GNSS *
    ublox_dir = FindPackageShare(package="ublox_gps").find("ublox_gps")
    ublox_launch_dir = os.path.join(ublox_dir, "launch")
    ublox_toggle = LaunchConfiguration("ublox_toggle")
    ublox_toggle_arg = DeclareLaunchArgument(
        name="ublox_toggle",
        default_value="True",
        description="Determines whether or not to start the Ublox ROS wrapper.",
    )
    ld.add_action(ublox_toggle_arg)
    # Launch Ublox ROS Wrapper
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_launch_dir, "ublox_gps_node-launch.py")
            ),
            condition=IfCondition(ublox_toggle),
        )
    )

    ntrip_toggle = LaunchConfiguration("ntrip_toggle")
    ntrip_toggle_arg = DeclareLaunchArgument(
        name="ntrip_toggle",
        default_value="False",
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
                os.path.join(pkg_share, "ntrip_client.launch.py")
            ),
            launch_arguments={
                "host": ntrip_host,
                "port": ntrip_port,
                "mountpoint": ntrip_mountpoint,
                "username": ntrip_username,
                "password": ntrip_password,
                "rtcm_message_package": "rtcm_msgs",
            }.items(),
            condition=IfCondition(ntrip_toggle),
        )
    )

    # * ROBOT LOCALIZATION *
    robot_localization_file_path = os.path.join(pkg_share, "config/ekf.yaml")
    # Start the navsat_tranform_node to convert our RTK fix to local frame
    ld.add_action(
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            remappings=[("/odometry/filtered", "/odometry/navsat_transformed")],
            parameters=[
                {
                    "magnetic_declination_radians": 0.0,
                    "yaw_offset": 0.0,
                    "zero_altitude": True,
                }
            ],
            condition=IfCondition(ublox_toggle),
        )
    )
    # Start robot localization using an Extended Kalman filter
    ld.add_action(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[robot_localization_file_path],
        )
    )

    # * RVIZ *
    rviz_config_file_path = "rviz/nav2_config.rviz"
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    # RVIZ Toggle
    rviz_toggle = LaunchConfiguration("rviz_toggle")
    rviz_toggle_arg = DeclareLaunchArgument(
        name="rviz_toggle",
        default_value="False",
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

    # * URDF, Joint State, Robot State *
    urdf_file_path = "models/basic_rmp.urdf.xacro"
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    # URDF Model Config
    urdf_model = LaunchConfiguration("urdf_model")
    urdf_model_arg = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_urdf_model_path,
        description="Absolute path to robot urdf file",
    )
    ld.add_action(urdf_model_arg)
    # Robot State Toggle
    robot_state_toggle = LaunchConfiguration("robot_state_toggle")
    robot_state_toggle_arg = DeclareLaunchArgument(
        name="robot_state_toggle",
        default_value="True",
        description="Whether to start the robot state publisher",
    )
    ld.add_action(robot_state_toggle_arg)
    # Joint State Toggle
    joint_state_toggle = LaunchConfiguration("joint_state_toggle")
    joint_state_toggle_arg = DeclareLaunchArgument(
        name="joint_state_toggle",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui",
    )
    ld.add_action(joint_state_toggle_arg)
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(["xacro ", urdf_model]),
                }
            ],
            condition=IfCondition(robot_state_toggle),
        )
    )
    # Publish the joint state values for the non-fixed joints in the URDF file.
    ld.add_action(
        Node(
            condition=IfCondition(joint_state_toggle),
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        )
    )

    # * NAV2 *
    nav2_dir = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_dir, "launch")
    # Params Config
    nav2_params_path = os.path.join(pkg_share, "params", "nav2_params.yaml")
    params_file = LaunchConfiguration("params_file")
    params_file_arg = DeclareLaunchArgument(
        name="params_file",
        default_value=nav2_params_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    ld.add_action(params_file_arg)
    # Map Config
    static_map_path = os.path.join(pkg_share, "maps", "empty_map.yaml")
    map_yaml_file = LaunchConfiguration("map")
    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=static_map_path,
        description="Full path to map file to load",
    )
    ld.add_action(map_arg)
    # Behavior Tree Config
    nav2_bt_path = FindPackageShare(package="nav2_bt_navigator").find(
        "nav2_bt_navigator"
    )
    behavior_tree_xml_path = os.path.join(
        nav2_bt_path, "behavior_trees", "navigate_w_replanning_and_recovery.xml"
    )
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    default_bt_xml_path_arg = DeclareLaunchArgument(
        name="default_bt_xml_filename",
        default_value=behavior_tree_xml_path,
        description="Full path to the behavior tree xml file to use",
    )
    ld.add_action(default_bt_xml_path_arg)
    # Namespace Config
    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        name="namespace", default_value="", description="Top-level namespace"
    )
    ld.add_action(namespace_arg)
    use_namespace = LaunchConfiguration("use_namespace")
    use_namespace_arg = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )
    ld.add_action(use_namespace_arg)
    # Nav2 Toggle
    nav_toggle = LaunchConfiguration("nav_toggle")
    nav_toggle_arg = DeclareLaunchArgument(
        name="nav_toggle",
        default_value="False",
        description="Toggle the nav2 stack on or off.",
    )
    ld.add_action(nav_toggle_arg)
    # Nav2 Autostart Config
    autostart_toggle = LaunchConfiguration("autostart_toggle")
    autostart_toggle_arg = DeclareLaunchArgument(
        name="autostart_toggle",
        default_value="False",
        description="Automatically startup the nav2 stack",
    )
    ld.add_action(autostart_toggle_arg)
    # Slam Config
    slam_toggle = LaunchConfiguration("slam_toggle")
    slam_toggle_arg = DeclareLaunchArgument(
        name="slam_toggle", default_value="False", description="Whether to run SLAM"
    )
    ld.add_action(slam_toggle_arg)
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    # Start Nav2
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, "bringup_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": use_namespace,
                "slam": slam_toggle,
                "map": map_yaml_file,
                "params_file": params_file,
                "default_bt_xml_filename": default_bt_xml_filename,
                "autostart": autostart_toggle,
            }.items(),
            condition=IfCondition(nav_toggle),
        )
    )

    # * STEERING *
    steering_toggle = LaunchConfiguration("steering_toggle")
    steering_toggle_arg = DeclareLaunchArgument(
        name="steering_toggle",
        default_value="False",
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

    # * ZED2i CAMERA *
    zed_dir = FindPackageShare(package="zed_wrapper").find("zed_wrapper")
    zed_launch_dir = os.path.join(zed_dir, "launch")
    zed_toggle = LaunchConfiguration("zed_toggle")
    zed_toggle_arg = DeclareLaunchArgument(
        name="zed_toggle",
        default_value="False",
        description="Determines whether or not to start the ZED 2i ROS wrapper.",
    )
    ld.add_action(zed_toggle_arg)
    svo_path = LaunchConfiguration("svo_path")
    svo_path_arg = DeclareLaunchArgument(
        "svo_path",
        default_value="live",  # 'live' used as patch for launch files not allowing empty strings as default parameters
        description="Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.",
    )
    ld.add_action(svo_path_arg)
    # Launch ZED 2i ROS Wrapper
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(zed_launch_dir, "zed2i.launch.py")
    #         ),
    #         condition=IfCondition(zed_toggle),
    #     )
    # )

    # ZED Wrapper node
    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("zed_wrapper"),
                    "/launch/include/zed_camera.launch.py",
                ]
            ),
            condition=IfCondition(zed_toggle),
            launch_arguments={
                "camera_model": "zed2i",
                "camera_name": "zed2i",
                "node_name": "zed_node",
                "config_common_path": os.path.join(
                    get_package_share_directory("zed_wrapper"), "config", "common.yaml"
                ),
                "config_camera_path": os.path.join(
                    get_package_share_directory("zed_wrapper"),
                    "config",
                    "zed2i.yaml",
                ),
                "publish_urdf": "true",
                "xacro_path": os.path.join(
                    get_package_share_directory("zed_wrapper"),
                    "urdf",
                    "zed_descr.urdf.xacro",
                ),
                "svo_path": svo_path,
                "base_frame": "base_footprint",
                "cam_pos_x": "0.3",
                "cam_pos_y": "0.0",
                "cam_pos_z": "0.2525",
                "cam_roll": "0.0",
                "cam_pitch": "0.0",
                "cam_yaw": "0.0",
            }.items(),
        )
    )
    # Pull a laserscan from the ZED Camera
    ld.add_action(
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            parameters=[
                {"range_min": 1.5, "range_max": 35.0, "output_frame": "camera_link"}
            ],
            remappings=[
                ("depth", "zed_2i/depth/image_raw"),
                ("depth_camera_info", "zed_2i/depth/camera_info"),
            ],
        )
    )

    # * Segway RMP *
    segway_toggle = LaunchConfiguration("segway_toggle")
    segway_toggle_arg = DeclareLaunchArgument(
        name="segway_toggle",
        default_value="True",
        description="Determines whether or not to start the Segway RMP ROS wrapper.",
    )
    segway_enable = LaunchConfiguration("segway_enable")
    segway_enable_arg = DeclareLaunchArgument(
        name="segway_enable",
        default_value="True",
        description="Determines whether or not to enable the Segway RMP.",
    )
    ld.add_action(segway_toggle_arg)
    ld.add_action(segway_enable_arg)
    # Launch Segway RMP ROS Wrapper
    ld.add_action(
        Node(
            package="segwayrmp",
            executable="SmartCar",
            remappings=[("/odom", "/odometry/wheel")],
            condition=IfCondition(segway_toggle),
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
            condition=IfCondition(segway_enable),
        )
    )

    return ld
