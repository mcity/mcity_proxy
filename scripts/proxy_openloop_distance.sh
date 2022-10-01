#!/bin/bash
ros2 service call /OpenLoopControllerDistance mcity_proxy_msgs/srv/OpenLoopControllerDistance "{meters_per_second: $1, meters: $2}"