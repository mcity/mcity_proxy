# mcity-proxy

## Controlling MAPP

You can find convenience scripts for controlling various aspects of the MAPP in the `scripts` directory.

Some highlights:

* `proxy_launch.sh`: starts the ROS nodes necessary to control the robot. Launch arguments can be passed to it in the form `arg:=value`
* `proxy_enable.sh`: enables the segway RMP. This must be run before the robot will respond to commands.
* `proxy_disable.sh`: disables the segway RMP. This will prevent the robot from being operated until it is re-enabled.
* `proxy_move_distance.sh`: Move the proxy forward in a straight line, at `$1` meters per second for `$2` meters.

## Domain IDs

Individual proxies and their base stations should be run with unique `ROS_DOMAIN_ID` variables set. These should be non-zero, since this is the default value and will pick up any other ROS nodes running on the network, and less than or equal to 101 in general (if all machines involved are running Linux, 215-232 inclusive can also safely be used). For more information, see [the docs](https://docs.ros.org/en/dashing/Concepts/About-Domain-ID.html)

For Mcity's autonomous proxies, we will use the convention of starting at `ROS_DOMAIN_ID=60` and incrementing by 1 for each additional proxy deployed.

