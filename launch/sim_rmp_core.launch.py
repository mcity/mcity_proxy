import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Constants for paths to different files and folders
    gazebo_models_path = "models"
    package_name = "mcity_proxy"
    robot_name_in_model = "rmp_401"
    rviz_config_file_path = "rviz/nav2_config.rviz"
    urdf_file_path = "models/basic_rmp.urdf.xacro"
    world_file_path = "worlds/smalltown.world"

    # Pose where we want to spawn the robot
    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "0.00"

    # Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    world_path = os.path.join(pkg_share, world_file_path)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    robot_localization_file_path = os.path.join(pkg_share, "config/ekf.yaml")
    nav2_dir = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_dir, "launch")
    static_map_path = os.path.join(pkg_share, "maps", "smalltown_world.yaml")
    nav2_params_path = os.path.join(pkg_share, "params", "nav2_params.yaml")
    nav2_bt_path = FindPackageShare(package="nav2_bt_navigator").find(
        "nav2_bt_navigator"
    )
    behavior_tree_xml_path = os.path.join(
        nav2_bt_path, "behavior_trees", "navigate_w_replanning_and_recovery.xml"
    )

    # Launch configuration variables specific to simulation
    autostart = LaunchConfiguration("autostart")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    slam = LaunchConfiguration("slam")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    namespace = LaunchConfiguration("namespace")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    urdf_model = LaunchConfiguration("urdf_model")
    use_namespace = LaunchConfiguration("use_namespace")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simulator = LaunchConfiguration("use_simulator")
    world = LaunchConfiguration("world")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name="namespace", default_value="", description="Top-level namespace"
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        name="autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        name="default_bt_xml_filename",
        default_value=behavior_tree_xml_path,
        description="Full path to the behavior tree xml file to use",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name="map",
        default_value=static_map_path,
        description="Full path to map file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name="params_file",
        default_value=nav2_params_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name="slam", default_value="False", description="Whether to run SLAM"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=default_rviz_config_path,
        description="Full path to the RVIZ config file to use",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Whether to execute gzclient",
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_urdf_model_path,
        description="Absolute path to robot urdf file",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name="use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name="use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name="use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model file to load",
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", urdf_model]),
            }
        ],
    )

    # Start the navsat_tranform_node to convert our RTK fix to local frame
    start_navsat_transform_cmd = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        remappings=[("/odometry/filtered", "/odometry/navsat_transformed")],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "magnetic_declination_radians": "0",
                "yaw_offset": "0",
                "zero_altitude": True,
            }
        ],
    )

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_file_path, {"use_sim_time": use_sim_time}],
    )

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        #     condition=UnlessCondition(gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={"world": world}.items(),
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
    )

    # Launch the robot
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            robot_name_in_model,
            "-topic",
            "robot_description",
            "-x",
            spawn_x_val,
            "-y",
            spawn_y_val,
            "-z",
            spawn_z_val,
            "-Y",
            spawn_yaw_val,
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "default_bt_xml_filename": default_bt_xml_filename,
            "autostart": autostart,
        }.items(),
    )

    # Steering controls
    start_robot_steering_cmd = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Pull a laserscan from the ZED Camera
    start_depthimage_to_laserscan_cmd = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "range_min": 1.5,
            "range_max": 35.0,
            "output_frame": "camera_link"
        }],
        remappings=[
            ('depth', 'zed_2i/depth/image_raw'),
            ('depth_camera_info', 'zed_2i/depth/camera_info')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_robot_steering_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(start_depthimage_to_laserscan_cmd)

    return ld
