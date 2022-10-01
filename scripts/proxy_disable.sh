#!/bin/bash
ros2 service call /set_chassis_enable segway_msgs/srv/RosSetChassisEnableCmd "{ros_set_chassis_enable_cmd: False}"