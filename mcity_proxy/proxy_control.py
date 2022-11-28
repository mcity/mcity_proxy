import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.srv import MoveDistanceSendGoal, MoveDistanceCancelGoal
from mcity_proxy_msgs.msg import (
    MoveDistanceFeedback,
    # MoveDistanceGoal,
    MoveDistanceResult,
)
from mcity_proxy_msgs.action import MoveDistance
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from mcity_proxy_msgs.action import MoveDistance
from segway_msgs.srv import RosSetChassisEnableCmd

import os
import time
import math
import copy
from threading import Thread, Lock
import socketio
import json


class SocketComms(socketio.ClientNamespace):
    proxy_control = None

    def __init__(self):
        self.api_key = os.environ.get("MCITY_OCTANE_KEY", "reticulatingsplines")
        self.id = os.environ.get("ROBOT_ID", 1)
        self.namespace = "/octane"
        self.channel = "proxy"
        super().__init__(self.namespace)

    def send_auth(self):
        """
        Emit an authentication event.
        """
        self.emit("auth", {"x-api-key": self.api_key}, namespace=self.namespace)

    # Define event callbacks
    def on_connect(self):
        """
        Handle connection event and send authentication key
        """
        self.send_auth()

    def on_join(self, data: dict):
        """
        Event fired when user joins a channel
        """
        # print('Subscribed to:', data)
        pass

    def on_channels(self, data: dict):
        """
        Event fired when a user requests current channel information.
        """
        self.emit("join", {"channel": self.channel}, namespace=self.namespace)

    def on_proxy(self, data: dict):
        """
        Event fired for each message on the 'proxy' topic
        """
        print("Recieved:", data)
        message_id = data.get("id", None)
        if message_id != self.id:
            print("This message isn't for me, ignoring")
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
        elif message_type == "service":
            # * Service
            print("No services currently supported, ignoring")
            return
        else:
            print("Unrecognized message type, ignoring")
            return

    def send_feedback(self, data: dict):
        """
        Send feedback over sockets
        """
        message = {"id": self.id, "type": "feedback", "data": json.dumps(data)}
        self.emit("proxy", message)

    def send_response(self, accepted: bool):
        """
        Send the action server's response to a goal
        """
        message = {
            "id": self.id,
            "type": "goal_response",
            "data": json.dumps({"accepted": accepted}),
        }
        self.emit("proxy", message)

    def send_result(self, data: dict):
        message = {"id": self.id, "type": "goal_result", "data": json.dumps(data)}
        self.emit("proxy", message)

    def send_cancel_response(self, accepted: bool):
        message = {
            "id": self.id,
            "type": "cancel_response",
            "data": json.dumps({"accepted": accepted}),
        }
        self.emit("proxy", message)

    def send_cancel_result(self, data: dict):
        message = {"id": self.id, "type": "cancel_result", "data": json.dumps(data)}
        self.emit("proxy", message)


class ProxyControl(Node):
    socket_comms = None

    def __init__(self):
        super().__init__("action_manager")

        # *Action Client
        self.ac_move_distance = ActionClient(self, MoveDistance, "move_distance")

        self.proxy_enable_service = self.create_client(
            RosSetChassisEnableCmd, "/set_chassis_enable"
        )

        self.estop_request = RosSetChassisEnableCmd.Request(
            ros_set_chassis_enable_cmd=False
        )

    def estop(self):
        future = self.proxy_enable_service(self.estop_request)
        rclpy.spin_until_future_complete(self, future)
        return self.future.result()

    def move_distance_send_goal(self, values):
        goal = values.move_distance_goal
        self.ac_move_distance.wait_for_server()
        self._send_goal_future_move_distance = (
            self._action_client_move_distance.send_goal_async(
                MoveDistance.Goal(
                    meters_per_second=goal.meters_per_second,
                    meters=goal.meters,
                ),
                feedback_callback=self.move_distance_feedback_callback,
            )
        )
        self._send_goal_future_move_distance.add_done_callback(
            self.move_distance_response_callback
        )

    def move_distance_cancel_goal(self):
        if self._goal_handle is not None:
            self._cancel_move_distance_goal = self._goal_handle.cancel_goal_async()
            self._cancel_move_distance_goal.add_done_callback(
                self.cancel_move_distance_finished_callback
            )
            self.socket_comms.send_cancel_response(True)
        else:
            self.socket_comms.send_cancel_response(False)
            pass

    def move_distance_feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback")
        self.socket_comms.send_feedback(feedback)

    def move_distance_response_callback(self, future):
        self.get_logger().info(f"Response Callback triggered")
        self._goal_handle = future.result()
        self.socket_comms.send_response(self._goal_handle.accepted)

    def move_distance_get_result_callback(self, future):
        self.get_logger().info(f"Result Callback triggered")
        result = future.result().result
        self._goal_handle = None
        self.socket_comms.send_result(self, result)

    def cancel_move_distance_finished_callback(self, future):
        self._goal_handle = None
        self.socket_comms.send_cancel_result(future.result())


def main(args=None):
    rclpy.init(args=args)

    proxy_control = ProxyControl()
    socket_comms = SocketComms()
    socket_comms.connect()
    server = os.environ.get("MCITY_OCTANE_SERVER", "wss://octane.mvillage.um.city/")

    sio = socketio.Client
    sio.register_namespace(socket_comms)
    sio.connect(server)

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
