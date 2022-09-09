# mcity-proxy

* `xacro basic_rmp.urdf.xacro > basic_rmp.urdf && gz sdf -p basic_rmp.urdf > basic_rmp_description/basic_rmp.sdf && gazebo`
* `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/dev_ws/src/mcity_proxy/models`
* `ros2 topic pub --once cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"`
* `rqt_robot_steering`
* `ros2 service call /set_chassis_enable segway_msgs/srv/RosSetChassisEnableCmd "{ros_set_chassis_enable_cmd: True}"`
* `sudo usermod -aG dialout $USER`, reboot or log out and back in
* `sudo apt-get purge modemmanager`

## Documentation

* ZED 2i 4mm focal length with Polarizer

## Domain IDs

Individual proxies and their base stations should be run with unique `ROS_DOMAIN_ID` variables set. These should be non-zero, since this is the default value and will pick up any other ROS nodes running on the network, and less than or equal to 101 in general (if all machines involved are running Linux, 215-232 inclusive can also safely be used). For more information, see [the docs](https://docs.ros.org/en/dashing/Concepts/About-Domain-ID.html)

For Mcity's autonomous proxies, we will use the convention of starting at `ROS_DOMAIN_ID=60` and incrementing by 1 for each additional proxy deployed.

## Bash Environment

Add the following to `~/.bashrc`

```
# Make a .bash_environment file to store machine specific, secret, and temporary variables
if [ -f "$HOME/.bash_environment" ]; then
	source $HOME/.bash_environment
fi
# Make a .container_environment file and place it in the root of a container to load container specific environment
if [ -f "/.container_environment" ]; then
	source /.container_environment
fi

# Machine Specific Settings and Environment Config
case "$HOSTNAME" in
	mcity-proxy-platform-* | mcity-proxy-basestation-*)
		if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
			source /usr/share/colcon_cd/function/colcon_cd.sh
			export _colcon_cd_root=/opt/ros/foxy
		fi
		if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
			source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
		fi
		export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/usr/local/zed"
		export ZED_DIR="/usr/local/zed/"
		for file in /opt/ros/foxy/setup.bash ~/local_ws/install/local_setup.bash ~/dev_ws/install/local_setup.bash; do
			if [ -f "$file" ]; then
				source $file
			fi
		done
		export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/dev_ws/src/mcity_proxy/models
		export PATH=$PATH:/home/luckierdodge/.local/bin
		export ROS_DOMAIN_ID=60
		;;
	*)
		:
		;;
esac
```

Add the following to `~/.bash_environment`, inserting the correct values

```
export NTRIP_USERNAME=""
export NTRIP_PASSWORD=""
```
