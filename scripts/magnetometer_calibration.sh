#!/bin/bash
ros2 run robot_calibration magnetometer_calibration --ros-args -p rotation_manual:=true -r /imu/mag:=/zed2i/zed_node/imu/mag
