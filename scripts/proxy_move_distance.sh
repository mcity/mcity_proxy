#!/bin/bash
ros2 action send_goal move_distance mcity_proxy_msgs/action/MoveDistance "{meters_per_second: $1, meters: $2}"
