import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import MoveCircle
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion

import time
import math
import copy
import numpy as np
from threading import Thread


class MoveCircleActionServer(Node):
    def __init__(self):
        super().__init__("move_circle_action_server")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.kP = 0.5
        self.latest_pose = 0
        self.latest_pose_x = 0
        self.latest_pose_y = 0
        self.latest_orientation_x = 0
        self.latest_orientation_y = 0
        self.latest_orientation_z = 0
        self.latest_orientation_w = 0
        self.callback_group = ReentrantCallbackGroup()
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            "/odometry/wheel",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self._action_server = ActionServer(
            self,
            MoveCircle,
            "move_circle",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

    def destroy(self):
        self._action_server.destory()
        super().destroy_node()

    def odom_callback(self, msg):
        self.latest_pose = copy.deepcopy(msg.pose.pose)
        self.latest_pose_x = copy.deepcopy(msg.pose.pose.position.x)
        self.latest_pose_y = copy.deepcopy(msg.pose.pose.position.y)
        self.latest_orientation_x = copy.deepcopy(msg.pose.pose.orientation.x)
        self.latest_orientation_y = copy.deepcopy(msg.pose.pose.orientation.y)
        self.latest_orientation_z = copy.deepcopy(msg.pose.pose.orientation.z)
        self.latest_orientation_w = copy.deepcopy(msg.pose.pose.orientation.w)

    def goal_callback(self, goal_request):
        self.get_logger().info("MoveCircle Action Server: Received Goal")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info("MoveCircle Action Server: Handling Accepted Goal")
        with self._goal_lock:
            # *This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # *Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info("MoveCircle Action Server: Cancelling Goal")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("MoveCircle Action Server: Executing Goal")

        start_time = time.time()
        feedback_msg = MoveCircle.Feedback()
        tick = start_time
        r = goal_handle.request.radius
        ms = goal_handle.request.meters_per_second
        c = 2 * np.pi * r
        angular_velocity = 2 * np.pi * (ms / c)
        while time.time() - start_time < goal_handle.request.seconds:
            result = self.check_goal_state_change(goal_handle)
            if result is not None:
                return result
            twist = Twist(
                linear=Vector3(x=ms),
                angular=Vector3(z=angular_velocity),
            )
            self.publisher_.publish(twist)

            if time.time() - tick > 1.0:  # * Send feedback every 1 second
                feedback_msg.current_position = Point(
                    x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
                )
                goal_handle.publish_feedback(feedback_msg)
                tick = time.time()
        # * endwhile

        twist = Twist()
        self.publisher_.publish(twist)

        goal_handle.succeed()
        result = MoveCircle.Result()
        result.final_position = Point(
            x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
        )

        return result

    def check_goal_state_change(self, goal_handle):
        result = None
        if not goal_handle.is_active:
            # *Another goal was set: abort, and return the current position
            self.get_logger().info("Goal aborted.")
            result = MoveCircle.Result()
            result.final_position = Point(
                x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
            )
        if goal_handle.is_cancel_requested:
            # *Goal was canceled, stop the robot and return the current position
            self.get_logger().info("Goal cancelled.")
            goal_handle.canceled()
            twist = Twist()
            self.publisher_.publish(twist)
            result = MoveCircle.Result()
            result.final_position = Point(
                x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
            )
        return result


def main(args=None):
    rclpy.init(args=args)

    move_circle_action_server = MoveCircleActionServer()
    multi_exec = MultiThreadedExecutor(num_threads=8)
    multi_exec.add_node(move_circle_action_server)
    multi_exec.spin()

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_circle_action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
