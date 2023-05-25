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

## `mcity` User Config

```
sudo usermod -aG dialout mcity
sudo usermod -aG i2c mcity
sudo usermod -aG gpio mcity
```

## Installation Notes

```
sudo pip3 install transforms3d
sudo apt install ros-foxy-tf-transformations -y
```

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

```

Make the file `~/.bash_environment` and add the following, filling in appropriate values for environment variables:

```
export NTRIP_USERNAME=
export NTRIP_PASSWORD=
export MCITY_OCTANE_KEY=
export ROBOT_ID=
export MCITY_OCTANE_SERVER="wss://octane.um.city"
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
export PATH=$PATH:/home/mcity/.local/bin
declare -i ROS_DOMAIN_ID=60+$ROBOT_ID
export ROS_DOMAIN_ID
```


## Wifi

* [Download this repo](https://github.com/RinCat/RTL88x2BU-Linux-Driver)
* `sudo make && sudo make install`
* Follow the instructions in the README to permanently enable USB 3.0 mode (must be plugged into 3.0 capable port!)


## DDS and Networking Shenanigans

```
sudo sysctl net.ipv4.ipfrag_time=3
sudo sysctl net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
#sudo apt install ros-foxy-rmw-cyclonedds-cpp
``

## Automatic Proxy Start

Run `sudo crontab -e` and add the following line:

```
@reboot /home/mcity/dev_ws/src/mcity_proxy/proxy_boot.sh
```

## Magnetometer Calibration

MAPP-02:

```
[INFO] [1684787286.957424315] [magnetometer_calibration]: Initial estimate for hard iron offsets: [5.
91344e-07, -7.124e-07, -4.03757e-06]
[INFO] [1684787287.544209454] [magnetometer_calibration]: Ceres Solver Report: Iterations: 22, Initia
l cost: 7.961363e+01, Final cost: 1.340365e-10, Termination: CONVERGENCE
[INFO] [1684787287.544424897] [magnetometer_calibration]: Estimated total magnetic field: 0.199024T
[INFO] [1684787287.544554181] [magnetometer_calibration]: You can compare to expected values from
[INFO] [1684787287.544652249] [magnetometer_calibration]:   https://www.ngdc.noaa.gov/geomag/calculat
ors/magcalc.shtml#igrfwmm
mag_bias_x: -0.114911
mag_bias_y: 0.114903
mag_bias_z: -0.114908
```
