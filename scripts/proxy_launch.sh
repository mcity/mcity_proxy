#!/bin/bash
# Bootstrap script for the proxy platform
sudo -l
#fastdds discovery -i 0 &
ros2 launch mcity_proxy proxy_platform.launch.py "$@"
