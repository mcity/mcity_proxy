import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import MoveDistance
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from segway_msgs.srv import RosSetChassisEnableCmd
from rclpy_message_converter import json_message_converter

import os
import time
import math
import copy
from threading import Thread, Lock
import socketio
import json
import time

class SocketComms(socketio.ClientNamespace):
    proxy_control = None

    def __init__(self):
        self.api_key = os.environ.get("MCITY_OCTANE_KEY", "reticulatingsplines")
        self.id = os.environ.get("ROBOT_ID", 1)
        self.namespace = "/octane"
        self.channel = "robot_proxy"
        self.room="robot"
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
        self.proxy_control.log(f'Subscribed to: {data}')

    def on_channels(self, data: dict):
        """
        Event fired when a user requests current channel information.
        """
        pass

    def on_auth_ok(self, data):
        self.proxy_control.log(f'Authorized, joining room {self.room}')
        self.emit("join", {"channel": self.room}, namespace=self.namespace)

    # def on_message(self, data):
    #     self.proxy_control.log(data)

    def on_robot_proxy(self, data: dict):
        """
        Event fired for each message on the 'robot_proxy' channel
        """
        self.proxy_control.log(f"Received: {data}")
        message_id = data.get("id", None)
        if str(message_id) != str(self.id):
            self.proxy_control.log("This message isn't for me, ignoring")
            return
        message_type = data.get("type", None)
        if message_type == "estop":
            # * Emergency Stop
            self.proxy_control.estop()
        elif message_type == "action":
            # * Action
            action = data.get("action", None)
            if action == "move_distance" or action == "/move_distance":
                # * Move Distance Action
                cancel = data.get("cancel", None)
                if cancel is not None and cancel:
                    # * Cancel current goal
                    self.proxy_control.move_distance_cancel_goal()
                values = data.get("values", None)
                if values is not None:
                    # * Send a new goal
                    self.proxy_control.move_distance_send_goal(values)
            else:
                self.proxy_control.log("Unknown action type, returning")
                return
        elif message_type == "service":
            # * Service
            self.proxy_control.log("No services currently supported, ignoring")
            return
        elif message_type == "topic":
            topic = data.get("topic", None)
            values = data.get("values", None)
            if topic == "cmd_vel" or topic == "/cmd_vel":
                self.proxy_control.publish_cmd_vel(values)
            else:
                self.proxy_control.log("Unsupported topic, returning")
                return
        else:
            self.proxy_control.log("Unrecognized message type, ignoring")
            return

    def send_ros_message(self, type, data):
        message = {
            "id": self.id,
            "type": type,
            "data": json_message_converter.convert_ros_message_to_json(data)
        }
        self.emit(self.channel, message)

    def send_message(self, type, data):
        message = {
            "id": self.id,
            "type": type,
            "data": json.dumps(data)
        }
        self.emit(self.channel, message)


class ProxyControl(Node):
    socket_comms = None

    def __init__(self):
        super().__init__("action_manager")

        # *Action Client
        self.ac_move_distance = ActionClient(self, MoveDistance, "move_distance")

        # *Services
        self.proxy_enable_service = self.create_client(
            RosSetChassisEnableCmd, "/set_chassis_enable"
        )

        self.estop_request = RosSetChassisEnableCmd.Request(
            ros_set_chassis_enable_cmd=False
        )

        # *Topics
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.goal_handle = None
        self.send_goal_future_move_distance = None
        self.cancel_move_distance_goal = None
        self.get_move_distance_result_future = None

    def log(self, message):
        self.get_logger().info(message)

    def estop(self):
        future = self.proxy_enable_service(self.estop_request)
        rclpy.spin_until_future_complete(self, future)
        return self.future.result()

    def publish_cmd_vel(self, values):
        msg = json_message_converter.convert_json_to_ros_message('geometry_msgs/Twist', values)
        self.cmd_vel_pub.publish(msg)


    def move_distance_send_goal(self, values):
        goal = values["move_distance_goal"]
        self.ac_move_distance.wait_for_server()
        self.send_goal_future_move_distance = (
            self.ac_move_distance.send_goal_async(
                MoveDistance.Goal(
                    meters_per_second=float(goal["meters_per_second"]),
                    meters=float(goal["meters"]),
                ),
                feedback_callback=self.move_distance_feedback_callback,
            )
        )
        self.send_goal_future_move_distance.add_done_callback(
            self.move_distance_response_callback
        )

    def move_distance_cancel_goal(self):
        if self.goal_handle is not None:
            self.cancel_move_distance_goal = self.goal_handle.cancel_goal_async()
            self.cancel_move_distance_goal.add_done_callback(
                self.cancel_move_distance_finished_callback
            )
            self.socket_comms.send_ros_message("cancel_response", True)
        else:
            self.socket_comms.send_ros_message("cancel_response", False)

    def move_distance_feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback Callback")
        self.socket_comms.send_ros_message("feedback", feedback.feedback)

    def move_distance_response_callback(self, future):
        self.get_logger().info(f"Response Callback triggered")
        self.goal_handle = future.result()
        self.socket_comms.send_message("goal_response", {"accepted": self.goal_handle.accepted})

    def move_distance_get_result_callback(self, future):
        # self.get_logger().info(f"Result Callback triggered")
        result = future.result().result
        self.goal_handle = None
        self.get_move_distance_result_future = None
        self.socket_comms.send_ros_message("goal_result", result)

    def cancel_move_distance_finished_callback(self, future):
        self.goal_handle = None
        self.get_move_distance_result_future = None
        self.socket_comms.send_ros_message("cancel_result", future.result())


def socket_io_spin(sio, socket_comms, proxy_control):
    proxy_control.log("Spinning socket client.")
    sio.wait()

def result_spin(sio, socket_comms, proxy_control):
    while True:
        if proxy_control.goal_handle is not None:
            if proxy_control.get_move_distance_result_future is None:
                proxy_control.get_move_distance_result_future = (
                    proxy_control.goal_handle.get_result_async()
                )
                proxy_control.get_move_distance_result_future.add_done_callback(
                    proxy_control.move_distance_get_result_callback
                )
        time.sleep(.1)


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

    socket_thread = Thread(target=socket_io_spin, args=(sio, socket_comms, proxy_control), daemon=True)
    socket_thread.start()

    result_thread = Thread(target=result_spin, args=(sio, socket_comms, proxy_control), daemon=True)
    result_thread.start()

    multi_exec = MultiThreadedExecutor(num_threads=8)
    multi_exec.add_node(proxy_control)
    multi_exec.spin()

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    proxy_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
