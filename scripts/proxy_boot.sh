#!/bin/bash
# Make sure you set NTRIP_USERNAME and NTRIP_PASSWORD in .bash_environment
for file in /opt/ros/foxy/setup.bash /home/mcity/local_ws/install/local_setup.bash /home/mcity/dev_ws/install/local_setup.bash /home/mcity/.bash_environment; do
	if [ -f "$file" ]; then
		source $file
	fi
done
ros2 launch mcity_proxy proxy_platform.launch.py
