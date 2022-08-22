# mcity-proxy

* `xacro basic_rmp.urdf.xacro > basic_rmp.urdf && gz sdf -p basic_rmp.urdf > basic_rmp_description/basic_rmp.sdf && gazebo`
* `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/dev_ws/src/mcity_proxy/models`
* `ros2 topic pub --once cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"`
* `rqt_robot_steering`
