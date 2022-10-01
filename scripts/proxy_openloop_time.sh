#!/bin/bash
ros2 service call /OpenLoopControllerTime mcity_proxy_msgs/srv/OpenLoopControllerTime "{meters_per_second: $1, seconds: $2}"