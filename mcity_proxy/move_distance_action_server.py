import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import MoveDistance
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion

import time
import math
import copy
import numpy as np
from threading import Thread

START_COURSE_CORRECT_THRESHOLD = 0.05
STOP_COURSE_CORRECT_THRESHOLD = 0.01


class MoveDistanceActionServer(Node):
    def __init__(self):
        super().__init__("move_distance_action_server")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.latest_pose = None
        self.kP = 0.5
        self.callback_group = ReentrantCallbackGroup()
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self._action_server = ActionServer(
            self,
            MoveDistance,
            "move_distance",
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
        # self.get_logger().info("ODOMETRY!")
        self.latest_pose = copy.deepcopy(msg.pose.pose)
        self.latest_pose_x = copy.deepcopy(msg.pose.pose.position.x)
        self.latest_pose_y = copy.deepcopy(msg.pose.pose.position.y)
        self.latest_orientation_x = copy.deepcopy(msg.pose.pose.orientation.x)
        self.latest_orientation_y = copy.deepcopy(msg.pose.pose.orientation.y)
        self.latest_orientation_z = copy.deepcopy(msg.pose.pose.orientation.z)
        self.latest_orientation_w = copy.deepcopy(msg.pose.pose.orientation.w)

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info("Received Cancel Request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        velocity, distance, direction = self.handle_negative_vel_and_dist(goal_handle)

        # * Capture our intitial state and extrapolate goal information
        twist = Twist(linear=Vector3(x=velocity))
        self.publisher_.publish(twist)
        while self.latest_pose is None:
            result = self.check_goal_state_change(goal_handle)
            if result is not None:
                return result
            self.get_logger().info("Waiting for starting odometry...")
            time.sleep(1)
        start_pose = copy.deepcopy(self.latest_pose)
        start_position = np.array([start_pose.position.x, start_pose.position.y, 0.0])
        feedback_msg = MoveDistance.Feedback()

        current_position = np.array([
            self.latest_pose.position.x,
            self.latest_pose.position.y,
            0.0,
        ])

        while np.linalg.norm(current_position - start_position) < 0.01:
            # * Make sure we haven't been cancelled or aborted
            result = self.check_goal_state_change(goal_handle)
            if result is not None:
                return result
            current_position = np.array([
                self.latest_pose.position.x,
                self.latest_pose.position.y,
                0.0,
            ])


        goal_heading = current_position - start_position
        goal_heading_unit = goal_heading / np.linalg.norm(goal_heading)
        goal_position = start_position + np.dot(distance, goal_heading_unit)
        previous_position = current_position.copy()

        self.get_logger().info(f"START POSITION: {start_position}")
        self.get_logger().info(f"GOAL POSITION: {goal_position}")

        tick = time.time()
        threshold = START_COURSE_CORRECT_THRESHOLD
        while np.linalg.norm(current_position - start_position) < np.abs(distance):
            # * Make sure we haven't been cancelled or aborted
            result = self.check_goal_state_change(goal_handle)
            if result is not None:
                return result

            current_position = np.array([
                self.latest_pose.position.x,
                self.latest_pose.position.y,
                0.0,
            ])

            if np.linalg.norm(current_position - previous_position) < 0.01:
                continue

            # * Figure out our current heading and the heading we want to be on


            current_heading = current_position - previous_position
            current_heading_unit = current_heading / np.linalg.norm(current_heading) * direction
            previous_position = current_position.copy()

            goal_heading = goal_position - current_position
            goal_heading_unit = goal_heading / np.linalg.norm(goal_heading)

            distance_remaining = np.abs(distance) - np.linalg.norm(
                current_position - start_position
            )
            theta = 0.0
            if distance_remaining > 0.5:
                angular_deflection = np.arccos(
                    np.clip(
                        np.dot(goal_heading_unit, current_heading_unit),
                        -1.0,
                        1.0,
                    )
                )
                if np.abs(angular_deflection) > (np.pi / 2):
                    self.get_logger().info(
                        f"Goal aborted. Too far off course. {angular_deflection}"
                    )
                    goal_handle.abort()
                    twist = Twist()
                    self.publisher_.publish(twist)
                    result = MoveDistance.Result()
                    result.final_position = Point(
                        x=self.latest_pose.position.x,
                        y=self.latest_pose.position.y,
                        z=0.0,
                    )
                    return result

                if np.abs(angular_deflection) > threshold:
                    if threshold == START_COURSE_CORRECT_THRESHOLD:
                        threshold = STOP_COURSE_CORRECT_THRESHOLD
                    theta = (  # * Attenuate our angular correction by kP
                        self.kP * angular_deflection * direction
                    )
                    # * Figure out which direction to turn
                    cross_product = np.cross(goal_heading_unit, current_heading_unit)
                    if cross_product[2] > 0.0:
                        theta *= -1.0
                else:
                    if threshold == STOP_COURSE_CORRECT_THRESHOLD:
                        threshold = START_COURSE_CORRECT_THRESHOLD

            twist.angular.z = theta
            self.publisher_.publish(twist)

            if time.time() - tick > 1.0:  # * Send feedback every 1 second
                feedback_msg.current_position = Point(
                    x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
                )
                goal_handle.publish_feedback(feedback_msg)
                if angular_deflection is not None:
                    self.get_logger().info(
                        f"{angular_deflection}, {goal_heading_unit}, {current_heading_unit}"
                    )
                tick = time.time()
        # * endwhile

        twist = Twist()
        self.publisher_.publish(twist)

        goal_handle.succeed()
        result = MoveDistance.Result()
        result.final_position = Point(
            x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
        )
        return result

    def handle_negative_vel_and_dist(self, goal_handle):
        velocity = float(goal_handle.request.meters_per_second)
        distance = float(goal_handle.request.meters)
        direction = 1
        if float(goal_handle.request.meters) < 0.0:
            direction = -1
            if float(goal_handle.request.meters_per_second) > 0.0:
                velocity *= -1
        elif float(goal_handle.request.meters_per_second) < 0.0:
            direction = -1
            distance *= -1
        return velocity, distance, direction

    def check_goal_state_change(self, goal_handle):
        result = None
        if not goal_handle.is_active:
            # Another goal was set: abort, and return the current position
            self.get_logger().info("Goal aborted.")
            result = MoveDistance.Result()
            result.final_position = Point(
                x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
            )
        if goal_handle.is_cancel_requested:
            # Goal was canceled, stop the robot and return the current position
            self.get_logger().info("Goal cancelled.")
            goal_handle.canceled()
            twist = Twist()
            self.publisher_.publish(twist)
            result = MoveDistance.Result()
            result.final_position = Point(
                x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
            )
        return result


def main(args=None):
    rclpy.init(args=args)

    move_distance_action_server = MoveDistanceActionServer()
    # odom_updater = OdomUpdater(move_distance_action_server)
    multi_exec = MultiThreadedExecutor(num_threads=8)
    multi_exec.add_node(move_distance_action_server)
    # multi_exec.add_node(odom_updater)
    multi_exec.spin()

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_distance_action_server.destroy()
    # odom_updater.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
