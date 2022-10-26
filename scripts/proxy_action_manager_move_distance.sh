#!/bin/bash
ros2 service call /action_manager/move_distance_send_goal mcity_proxy_msgs/srv/MoveDistanceSendGoal "{move_distance_goal: {meters_per_second: $1, meters: $2}}"
