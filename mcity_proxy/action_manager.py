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

import time
import math
import copy
from threading import Thread, Lock


class ActionManager(Node):
    states = [
        "AWAITING_GOAL",
        "AWAITING_RESPONSE",
        "AWAITING_RESULT",
        "CANCELLING_GOAL",
        "HALT"
    ]

    def __init__(self):
        super().__init__("action_manager")

        # *Action Client
        self._action_client_move_distance = ActionClient(
            self, MoveDistance, "move_distance"
        )

        # *Publishers
        self.pub_move_distance_feedback = self.create_publisher(
            MoveDistanceFeedback, "action_manager/move_distance_feedback", 10
        )
        self.pub_move_distance_result = self.create_publisher(
            MoveDistanceResult, "action_manager/move_distance_result", 10
        )

        # *Locks and Threads
        self.goal_lock = Lock()
        self.dispatch_thread = Thread(target=self.dispatcher_loop)

        # *Initialize State
        self.reset_state()

        # *Subscribers
        # self.sub_move_distance_goal = self.create_subscription()

        # *Services
        self.srv_move_distance_goal = self.create_service(
            MoveDistanceSendGoal,
            "action_manager/move_distance_send_goal",
            self.move_distance_send_goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.srv_move_distance_cancel = self.create_service(
            MoveDistanceCancelGoal,
            "action_manager/move_distance_cancel_goal",
            self.move_distance_cancel_goal_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.dispatch_thread.start()

    def reset_state(self):
        with self.goal_lock:
            self.get_logger().info("RESET STATE")
            self.state = "AWAITING_GOAL"
            self.goal = None
            self.accepted = None
            self.result = None
            self._goal_handle = None

    def move_distance_send_goal_callback(self, request, response):
        response.accepted = False
        if self.state == "AWAITING_GOAL":
            self.get_logger().info(
                f"Received request: {request.move_distance_goal}, dispatching..."
            )
            with self.goal_lock:
                self.goal = request.move_distance_goal
            stopwatch = time.time()
            while self.state != "AWAITING_RESULT":
                if stopwatch - time.time() > 10:
                    break
            if self.accepted is not None:
                response.accepted = self.accepted
                if not self.accepted:
                    self.reset_state()
            else:
                self.get_logger().info(f"Timed out waiting for response.")
        else:
            self.get_logger().info(f"Goal in progress {self.goal}, rejecting...")
        return response

    def dispatcher_loop(self):
        stopwatch = None
        while True:
            # self.get_logger().info(f"Dispatch loop")
            with self.goal_lock:
                if self.state == "AWAITING_GOAL" and self.goal is not None:
                    self.get_logger().info(f"{self.state}, {self.goal}")
                    self._action_client_move_distance.wait_for_server(timeout_sec=5)
                    self.get_logger().info(f"Dispatcher: Found Action Server")

                    self._send_goal_future_move_distance = (
                        self._action_client_move_distance.send_goal_async(
                            MoveDistance.Goal(meters_per_second=self.goal.meters_per_second, meters=self.goal.meters),
                            feedback_callback=self.move_distance_feedback_callback,
                        )
                    )
                    self._send_goal_future_move_distance.add_done_callback(
                        self.move_distance_response_callback
                    )
                    self.get_logger().info(f"Dispatcher: Sent Goal")
                    stopwatch = time.time()
                    self.state = "AWAITING_RESPONSE"

            if self.state == "AWAITING_RESPONSE" and self._goal_handle is not None:
                self.get_logger().info(f"Dispatcher: Recieved response from Action Server")
                stopwatch = None
                self.accepted = self._goal_handle.accepted
                if self.accepted:
                    self.get_logger().info(f"Dispatcher: Goal accepted, executing...")
                    self._get_move_distance_result_future = (
                        self._goal_handle.get_result_async()
                    )
                    self._get_move_distance_result_future.add_done_callback(
                        self.move_distance_get_result_callback
                    )
                    self.state = "AWAITING_RESULT"
                else:
                    self.get_logger().info(f"Dispatcher: Goal not accepted, resetting...")
                    self.reset_state()


            if self.state == "CANCELLING_GOAL":
                self.get_logger().info(f"Dispatcher: Cancelling...")
                if self._goal_handle is not None:
                    self._cancel_move_distance_goal = self._goal_handle.cancel_goal_async()
                    # self._cancel_move_distance_goal.add_done_callback(
                    #     self.cancel_move_distance_finished_callback
                    # )
                    self.state = "HALT"
                else:
                    self.reset_state()

            if self.state == "AWAITING_RESPONSE" and self._goal_handle is None and stopwatch is not None:
                if time.time() - stopwatch > 10:
                    self.get_logger().info(f"Dispatcher: Timed out waiting for response from Action Server...")
                    stopwatch = None
                    self.reset_state()
            continue

    def move_distance_feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback")
        self.pub_move_distance_feedback.publish(feedback)

    def move_distance_response_callback(self, future):
        self.get_logger().info(f"Response Callback triggered")
        self._goal_handle = future.result()

    def move_distance_get_result_callback(self, future):
        self.get_logger().info(f"Result Callback triggered")
        self.reset_state()
        result = future.result().result
        self.pub_move_distance_result.publish(result)

    def move_distance_cancel_goal_callback(self, request, response):
        with self.goal_lock:
            if self.goal is not None:
                self.get_logger().info(f"Cancelling goal...")
                self.state = "CANCELLING_GOAL"
                response.accepted = True
            else:
                self.get_logger().info(f"No active goal...")
                response.accepted = False
            return response

    # def cancel_move_distance_finished_callback(self, future):
    #     self.reset_state()


def main(args=None):
    rclpy.init(args=args)

    action_manager = ActionManager()

    multi_exec = MultiThreadedExecutor(num_threads=8)
    multi_exec.add_node(action_manager)
    multi_exec.spin()

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    action_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
