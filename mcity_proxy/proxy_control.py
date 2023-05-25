import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import MoveDistance, MoveCircle, WaypointNav
from mcity_proxy_msgs.msg import Waypoint
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from segway_msgs.srv import RosSetChassisEnableCmd
from segway_msgs.msg import BmsFb
from rclpy_message_converter import json_message_converter
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint

import os
import time
import math
import copy
from threading import Thread, Lock, Event
import socketio
import json
import time
import re
import numpy as np

shutdown_event = Event()

class SocketComms(socketio.ClientNamespace):
    proxy_control = None

    def __init__(self):
        self.api_key = os.environ.get("MCITY_OCTANE_KEY", "reticulatingsplines")
        self.id = os.environ.get("ROBOT_ID", 1)
        self.beacon_id = os.environ.get("BEACON_ID", "MAPP0" + self.id)
        self.namespace = "/octane"
        self.channel = "robot_proxy"
        self.room = "robot"
        self.scenario = None
        self.scenario_type = None
        super().__init__(self.namespace)

    def send_auth(self):
        """
        Emit an authentication event.
        """
        self.proxy_control.log("Sending Auth")
        self.emit("auth", {"x-api-key": self.api_key}, namespace=self.namespace)

    # Define event callbacks
    def on_connect(self):
        """
        Handle connection event and send authentication key
        """
        self.proxy_control.log("Connected")
        self.send_auth()

    def on_join(self, data: dict):
        """
        Event fired when user joins a channel
        """
        self.proxy_control.log(f"Subscribed to: {data}")

    def on_channels(self, data: dict):
        """
        Event fired when a user requests current channel information.
        """
        pass

    def on_auth_ok(self, data):
        self.proxy_control.log(f"Authorized, joining room {self.room}")
        self.emit("join", {"channel": self.room}, namespace=self.namespace)

    def on_robot_proxy(self, data: dict):
        """
        Event fired for each message on the 'robot_proxy' channel
        """
        message_id = data.get("id", None)
        if str(message_id) != str(self.id):
            return
        message_type = data.get("type", None)
        if message_type == "estop":
            # * Emergency Stop
            return self.proxy_control.estop()
        elif message_type == "enable":
            return self.proxy_control.enable()
        elif message_type == "disable":
            return self.proxy_control.disable()
        elif message_type == "action":
            # * Action
            return self.handle_action(data)
        elif message_type == "service":
            # * Service
            return self.handle_service(data)
        elif message_type == "topic":
            # * Topic
            return self.handle_topic(data)
        elif message_type == "set_scenario":
            self.scenario_type = data.get("scenario_type", None)
            self.scenario = data.get("scenario", None)
            return True
        elif message_type == "get_scenario":
            if self.scenario is None:
                return False
            return self.scenario
        elif message_type == "running":
            return self.proxy_control.is_running
        elif message_type == "run":
            if self.scenario is None:
                return False
            self.scenario["id"] = self.id
            return self.on_robot_proxy(data=self.scenario)
        elif message_type == "get_state":
            return {
                "latitude": self.proxy_control.last_navsat_fix.latitude,
                "longitude": self.proxy_control.last_navsat_fix.longitude,
                "running": self.proxy_control.is_running,
                "disabled": self.proxy_control.is_disabled(),  # Is the robot restricted from computer control override or e-stop
            }
        elif message_type == "get_batt":
            if self.proxy_control.last_bms_fb is None:
                return False
            return {
                "charging": self.proxy_control.last_bms_fb.bat_charging == 1,
                "percent": self.proxy_control.last_bms_fb.bat_soc,
                "voltage": self.proxy_control.last_bms_fb.bat_vol,
            }
        elif message_type == "get_conn":
            try:
                iwout = os.popen("iwconfig").read()
                # * Below is what we in the business call "Magical regex that'll find the thing you need"
                m = re.findall(
                    "(wlan[0-9]+).*?Signal level=([0-9/]+)", iwout, re.DOTALL
                )
                return {
                    "type": "WIFI",  # DSRC, WIFI, CELLULAR, ZIGBEE, ETHERNET, DIRECT
                    "strength": m[0][1],
                    "network": os.popen("iwgetid -r").read()[:-1],
                }
            except:
                return False
        else:
            return

    def handle_action(self, data: dict):
        self.proxy_control.log(f"Received: {data}")
        cancel = data.get("cancel", None)
        if cancel is not None and cancel:
            # * Cancel current goal
            return self.proxy_control.cancel_goal()
        action = data.get("action", None)
        if action == "move_distance" or action == "/move_distance":
            # * Move Distance Action
            values = data.get("values", None)
            if values is not None:
                # * Send a new goal
                return self.proxy_control.move_distance_send_goal(values)
            else:
                self.proxy_control.log("No values provided.")
                return "Please provide a velocity and distance"
        elif action == "move_circle" or action == "/move_circle":
            # * Move Distance Action
            values = data.get("values", None)
            if values is not None:
                # * Send a new goal
                return self.proxy_control.move_circle_send_goal(values)
            else:
                self.proxy_control.log("No values provided.")
                return "Please provide a velocity and distance"
        elif action == "waypoint_nav" or action == "/waypoint_nav":
            # * Execute Waypoint Navigation Action
            values = data.get("values", None)
            if values is not None:
                # * Send a new goal
                return self.proxy_control.waypoint_nav_send_goal(values)
            else:
                self.proxy_control.log("No waypoints provided.")
                return "Please provide an array of waypoints"
        else:
            self.proxy_control.log("Unknown action type, returning")
            return "Unknown action type"

    def handle_service(self, data: dict):
        self.proxy_control.log(f"Received: {data}")
        service = data.get("service", None)
        self.proxy_control.log("Service not currently supported, ignoring")
        return "Unsupported Service"

    def handle_topic(self, data: dict):
        self.proxy_control.log(f"Received: {data}")
        topic = data.get("topic", None)
        values = data.get("values", None)
        if topic == "cmd_vel" or topic == "/cmd_vel":
            return self.proxy_control.publish_cmd_vel(values)
        else:
            self.proxy_control.log("Unsupported topic, returning")
            return "Unsupported Topic"

    def send_ros_message(self, message_type, data):
        message = {
            "id": self.id,
            "type": message_type,
            "data": json_message_converter.convert_ros_message_to_json(data),
        }
        self.emit(self.channel, message)

    def send_message(self, type, data):
        message = {"id": self.id, "type": type, "data": json.dumps(data)}
        self.emit(self.channel, message)

    def status_update(self):
        message = {
            "id": self.id,
            "type": "proxy_status",
            "data": {
                "navsat_fix": json_message_converter.convert_ros_message_to_json(
                    self.proxy_control.last_navsat_fix
                ),
                "battery_state": json_message_converter.convert_ros_message_to_json(
                    self.proxy_control.last_bms_fb
                ),
            },
        }
        self.emit(self.channel, message)
        # *Impersonate a Beacon for Position updates
        heading = self.proxy_control.last_odom.pose.pose.orientation.z * 180 / np.pi
        if heading < 0:
            heading += 360
        message = {
            "id": self.beacon_id,
            "payload": {
                "state": {
                    "dynamics": {
                        "longitude": float(
                            self.proxy_control.last_navsat_fix.longitude
                        ),
                        "latitude": float(self.proxy_control.last_navsat_fix.latitude),
                        "heading": round(heading, 2),
                        "velocity": round(
                            np.linalg.norm(
                                [
                                    self.proxy_control.last_odom.twist.twist.linear.x,
                                    self.proxy_control.last_odom.twist.twist.linear.y,
                                    self.proxy_control.last_odom.twist.twist.linear.z,
                                ]
                            ),
                            2,
                        ),
                        # "acceleration": 0,
                        "elevation": self.proxy_control.last_navsat_fix.altitude,
                        # Format is 2022-10-20T13:09:21.422Z
                        # "updated": reading_taken_dt.isoformat(sep='T', timespec='milliseconds')
                    }
                }
            },
        }
        self.emit("beacon_message", message, namespace="/octane")


class ProxyControl(Node):
    socket_comms = None

    def __init__(self):
        super().__init__("action_manager")

        # *Action Clients
        self.ac_move_distance = ActionClient(self, MoveDistance, "move_distance")
        self.ac_move_circle = ActionClient(self, MoveCircle, "move_circle")
        self.ac_waypoint_nav = ActionClient(self, WaypointNav, "waypoint_nav")

        # *Services
        self.proxy_enable_service = self.create_client(
            RosSetChassisEnableCmd, "/set_chassis_enable"
        )

        self.disable_request = RosSetChassisEnableCmd.Request(
            ros_set_chassis_enable_cmd=False
        )
        self.enable_request = RosSetChassisEnableCmd.Request(
            ros_set_chassis_enable_cmd=True
        )

        # *Topic Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # *Topic Subscribers
        self.navsat_sub = self.create_subscription(
            NavSatFix, "/fix", self.navsat_fix_callback, 10
        )
        self.bms_fb_sub = self.create_subscription(
            BmsFb, "/bms_fb", self.bms_fb_callback, 10
        )
        self.odometry_filtered_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_filtered_callback, 10
        )

        # *Local Data Members
        self.reset()
        self.is_running = False
        self.last_navsat_fix = NavSatFix()
        self.last_bms_fb = BmsFb()
        self.last_odom = Odometry()

    def reset(self):
        self.is_running = False
        self.goal_handle = None
        self.send_goal_future_move_distance = None
        self.send_goal_future_move_circle = None
        self.send_goal_future_waypoint_nav = None
        self.cancel_move_distance_goal = None
        self.cancel_waypoint_nav_goal = None
        self.get_move_distance_result_future = None
        self.get_waypoint_nav_result_future = None

    def log(self, message):
        self.get_logger().info(message)

    def estop(self):
        """
        Immediately disable the robot
        """
        self.cmd_vel_pub.publish(Twist())
        self.proxy_enable_service.call(self.disable_request)
        self.reset()
        return True

    def enable(self):
        """
        Enable the Segway RMP
        """
        return self.proxy_enable_service.call(self.enable_request)

    def disable(self):
        """
        Disable the Segway RMP
        """
        return self.proxy_enable_service.call(self.disable_request)

    def is_disabled(self):
        # TODO
        return False

    def publish_cmd_vel(self, values):
        """
        Broadcast a cmd_vel topic received from the server as a
        ROS message
        """
        msg = json_message_converter.convert_json_to_ros_message(
            "geometry_msgs/Twist", values
        )
        self.cmd_vel_pub.publish(msg)

    def navsat_fix_callback(self, msg):
        """
        Update our GPS position estimate (used in our state updates)
        """
        self.last_navsat_fix = msg

    def bms_fb_callback(self, msg):
        """
        Update our battery status info (used in our state updates)
        """
        self.last_bms_fb = msg

    def odometry_filtered_callback(self, msg):
        """
        Update our odometry status info (used in beacon updates)
        """
        self.last_odom = msg

    def feedback_callback(self, feedback):
        """
        Called when we have feedback from an action server
        """
        self.socket_comms.send_ros_message("feedback", feedback.feedback)

    def move_distance_send_goal(self, values):
        """
        Handle a new MoveDistance goal from the server
        """
        self.is_running = True
        goal = values["move_distance_goal"]
        self.ac_move_distance.wait_for_server()
        self.send_goal_future_move_distance = self.ac_move_distance.send_goal_async(
            MoveDistance.Goal(
                meters_per_second=float(goal["meters_per_second"]),
                meters=float(goal["meters"]),
            ),
            feedback_callback=self.feedback_callback,
        )
        self.send_goal_future_move_distance.add_done_callback(self.response_callback)

    def move_circle_send_goal(self, values):
        """
        Handle a new MoveCircle goal from the server
        """
        self.is_running = True
        goal = values["move_circle_goal"]
        self.ac_move_circle.wait_for_server()
        self.send_goal_future_move_circle = self.ac_move_circle.send_goal_async(
            MoveCircle.Goal(
                meters_per_second=float(goal["meters_per_second"]),
                radius=float(goal["radius"]),
                seconds=float(goal["seconds"]),
            ),
            feedback_callback=self.feedback_callback,
        )
        self.send_goal_future_move_circle.add_done_callback(self.response_callback)

    def waypoint_nav_send_goal(self, values):
        """
        Handle a new WaypointNav goal from the server
        """
        self.is_running = True
        goal = values.get("waypoint_nav_goal", None)
        self.ac_waypoint_nav.wait_for_server()
        waypoints = []
        for waypoint in goal["waypoints"]:
            waypoints.append(
                Waypoint(
                    lat_long=GeoPoint(
                        latitude=waypoint["latitude"],
                        longitude=waypoint["longitude"],
                        altitude=0.0,
                    ),
                    velocity=float(waypoint["meters_per_second"]),
                )
            )
        self.send_goal_future_waypoint_nav = self.ac_waypoint_nav.send_goal_async(
            WaypointNav.Goal(waypoints=waypoints),
            feedback_callback=self.feedback_callback,
        )
        self.send_goal_future_waypoint_nav.add_done_callback(self.response_callback)

    def cancel_goal(self):
        """
        Handle a goal cancellation from the server
        """
        if self.goal_handle is not None:
            self.cancel_move_distance_goal = self.goal_handle.cancel_goal_async()
            self.cancel_move_distance_goal.add_done_callback(
                self.cancel_finished_callback
            )
            self.socket_comms.send_message("cancel_response", True)
        else:
            self.socket_comms.send_message("cancel_response", False)

    def response_callback(self, future):
        """
        Called when our action server finishes processing a goal (i.e. accepted or rejected)
        """
        self.goal_handle = future.result()
        self.socket_comms.send_message(
            "goal_response", {"accepted": self.goal_handle.accepted}
        )

    def get_result_callback(self, future):
        """
        Called when we complete a goal
        """
        result = future.result().result
        self.get_logger().info("Sending goal result.")
        self.socket_comms.send_ros_message("goal_result", result)
        self.reset()

    def cancel_finished_callback(self, future):
        """
        Called when we finish cancelling a goal
        """
        self.socket_comms.send_ros_message("cancel_result", future.result())
        self.reset()


def socket_io_spin(sio, socket_comms, proxy_control):
    """
    Keep our socketio client running indefinitely
    """
    proxy_control.log("Spinning socket client.")
    sio.wait()


def result_spin(sio, socket_comms, proxy_control):
    """
    This is a workaround to deal with a limitation in rclpy
    that prevents adding callbacks within other callbacks
    """
    while True:
        if proxy_control.goal_handle is not None:
            if proxy_control.get_move_distance_result_future is None:
                proxy_control.get_move_distance_result_future = (
                    proxy_control.goal_handle.get_result_async()
                )
                proxy_control.get_move_distance_result_future.add_done_callback(
                    proxy_control.get_result_callback
                )
        time.sleep(0.1)
        if shutdown_event.is_set():
            break


def status_updater(sio, socket_comms, proxy_control):
    while True:
        try:
            socket_comms.status_update()
        except Exception as e:
            proxy_control.get_logger().debug(str(e))
        time.sleep(1.0)  # * Update status once per second
        if shutdown_event.is_set():
            break


def main(args=None):
    rclpy.init(args=args)

    proxy_control = ProxyControl()
    socket_comms = SocketComms()
    socket_comms.proxy_control = proxy_control
    proxy_control.socket_comms = socket_comms
    server = os.environ.get("MCITY_OCTANE_SERVER", "wss://octane.mvillage.um.city/")

    sio = socketio.Client()
    sio.register_namespace(namespace_handler=socket_comms)
    sio.connect(server)

    socket_thread = Thread(
        target=socket_io_spin, args=(sio, socket_comms, proxy_control), daemon=True
    )
    socket_thread.start()

    result_thread = Thread(
        target=result_spin, args=(sio, socket_comms, proxy_control), daemon=True
    )
    result_thread.start()

    status_thread = Thread(
        target=status_updater, args=(sio, socket_comms, proxy_control), daemon=True
    )
    status_thread.start()

    multi_exec = MultiThreadedExecutor(num_threads=8)
    multi_exec.add_node(proxy_control)
    multi_exec.spin()

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    proxy_control.destroy_node()
    shutdown_event.set()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
