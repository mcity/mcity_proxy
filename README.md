# mcity-proxy

* `xacro basic_rmp.urdf.xacro > basic_rmp.urdf && gz sdf -p basic_rmp.urdf > basic_rmp_description/basic_rmp.sdf && gazebo`
* `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/dev_ws/src/mcity_proxy/models`
* `ros2 topic pub --once cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"`
* `rqt_robot_steering`

## Documentation

* ZED 2i 4mm focal length with Polarizer

## Domain IDs

Individual proxies and their base stations should be run with unique `ROS_DOMAIN_ID` variables set. These should be non-zero, since this is the default value and will pick up any other ROS nodes running on the network, and less than or equal to 101 in general (if all machines involved are running Linux, 215-232 inclusive can also safely be used). For more information, see [the docs](https://docs.ros.org/en/dashing/Concepts/About-Domain-ID.html)

For Mcity's autonomous proxies, we will use the convention of starting at `ROS_DOMAIN_ID=60` and incrementing by 1 for each additional proxy deployed.

## Bash Environment

```
if [[ -z "$CONTAINER_NAME" ]]; then
    # We're not in a container
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/foxy
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/usr/local/zed"
    export ZED_DIR="/usr/local/zed/"
    source ~/dev_ws/install/local_setup.bash
    source /opt/ros/foxy/setup.bash
else
    # We're in a container
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/foxy
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    #export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/usr/local/zed"
    #export ZED_DIR="/usr/local/zed/"
    source /docker_ws/install/local_setup.bash
fi

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/dev_ws/src/mcity_proxy/models
export PATH=$PATH:/home/luckierdodge/.local/bin
export ROS_DOMAIN_ID=60
```
