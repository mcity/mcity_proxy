import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import WaypointNav
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint

import time
import math
import copy
import numpy as np
from threading import Thread

COURSE_CORRECT_THRESHOLD = 0.01
COURSE_CORRECTION_CUTOFF = 0.25
ANGULAR_DEFLECTION_ABORT = 2 * np.pi / 3
GOAL_POSITION_TOLERANCE = 2.0


class WaypointNavActionServer(Node):
    def __init__(self):
        super().__init__("waypoint_nav_action_server")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.goal_position = np.array([0.0, 0.0, 0.0])
        self.latest_pose = None
        self.kP = 0.5
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            WaypointNav,
            "waypoint_nav",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # *Subscriptions and Publishers
        self.odom_subscriber_ = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self.to_ll_service_ = self.create_client(ToLL, "/toLL")
        self.from_ll_service_ = self.create_client(FromLL, "/fromLL")

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def odom_callback(self, msg):
        self.latest_pose = copy.deepcopy(msg.pose.pose)

    def goal_callback(self, goal_request):
        self.get_logger().info("WaypointNav Action Server: Received Goal")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info("WaypointNav Action Server: Handling Accepted Goal")
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info("WaypointNav Action Server: Cancelling Goal")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("WaypointNav Action Server: Executing Goal")
        waypoints = goal_handle.request.waypoints

        for pt in waypoints:
            # * Make sure we haven't been cancelled or aborted
            result = self.check_goal_state_change(self._goal_handle)
            if result is not None:
                return result
            result = self.navigate_to_target(pt)
            if result is not None:
                return result

        self._goal_handle.succeed()
        result = WaypointNav.Result()
        result.final_position = self.point_to_lat_long(
            x=self.latest_pose.position.x, y=self.latest_pose.position.y
        )
        return result

    def point_to_lat_long(self, x, y):
        self.future = self.to_ll_service_.call_async(
            ToLL.Request(map_point=Point(x=float(x), y=float(y), z=0.0))
        )
        rclpy.spin_until_future_complete(self, self.future)
        lat_long = self.future.result().ll_point
        return GeoPoint(
            latitude=lat_long.latitude, longitude=lat_long.longitude, altitude=0.0
        )

    def lat_long_to_point(self, lat_long):
        self.future = self.from_ll_service_.call_async(
            FromLL.Request(ll_point=lat_long)
        )
        rclpy.spin_until_future_complete(self, self.future)
        point = self.future.result().map_point
        return np.array([point.x, point.y, 0])

    def navigate_to_target(self, waypoint):
        self.goal_position = self.lat_long_to_point(waypoint.lat_long)

        # * Capture our initial state and extrapolate goal information
        twist = Twist(linear=Vector3(x=waypoint.velocity))
        self.publisher_.publish(twist)
        feedback_msg = WaypointNav.Feedback()

        current_position = np.array(
            [
                self.latest_pose.position.x,
                self.latest_pose.position.y,
                0.0,
            ]
        )

        self.get_logger().info(str(current_position))
        self.get_logger().info(str(self.goal_position))

        tick = time.time()
        while (
            np.linalg.norm(current_position - self.goal_position)
            > GOAL_POSITION_TOLERANCE
        ):  # *Loop until we've arrived
            # *Make sure we haven't been cancelled or aborted
            result = self.check_goal_state_change(self._goal_handle)
            if result is not None:
                return result

            # *Update Current Position and Orientation
            current_position = np.array(
                [
                    self.latest_pose.position.x,
                    self.latest_pose.position.y,
                    0.0,
                ]
            )
            yaw = euler_from_quaternion(
                [
                    self.latest_pose.orientation.x,
                    self.latest_pose.orientation.y,
                    self.latest_pose.orientation.z,
                    self.latest_pose.orientation.w,
                ]
            )[2]

            # *Calculate how much we need to turn to correct our heading (multiply by 2 to make it faster)
            theta = float(
                2
                * (
                    np.arctan2(
                        self.goal_position[1] - current_position[1],
                        self.goal_position[0] - current_position[0],
                    )
                    - yaw
                )
            )

            if theta > COURSE_CORRECT_THRESHOLD and (
                np.linalg.norm(current_position - self.goal_position)
                > COURSE_CORRECTION_CUTOFF
            ):  # *Only turn if we're not close to the goal and sufficiently off course (to prevent over steering)
                twist.angular.z = float(theta)
            else:
                twist.angular.z = float(0.0)
            self.publisher_.publish(twist)

            if time.time() - tick > 1.0:  # * Send feedback every 1 second
                self.get_logger().info("Feedback")
                feedback_msg.current_position = self.point_to_lat_long(
                    x=self.latest_pose.position.x, y=self.latest_pose.position.y
                )
                self._goal_handle.publish_feedback(feedback_msg)
                tick = time.time()
        # * end while

        twist = Twist()
        self.publisher_.publish(twist)

    def check_goal_state_change(self, goal_handle):
        result = None
        if not goal_handle.is_active:
            # *Another goal was set: abort, and return the current position
            self.get_logger().info("Goal aborted.")
            result = WaypointNav.Result()
            result.final_position = self.point_to_lat_long(
                x=self.latest_pose.position.x, y=self.latest_pose.position.y
            )
        if goal_handle.is_cancel_requested:
            # *Goal was canceled, stop the robot and return the current position
            self.get_logger().info("Goal cancelled.")
            goal_handle.canceled()
            twist = Twist()
            self.publisher_.publish(twist)
            result = WaypointNav.Result()
            result.final_position = self.point_to_lat_long(
                x=self.latest_pose.position.x, y=self.latest_pose.position.y
            )

        return result


def main(args=None):
    rclpy.init(args=args)

    waypoint_nav_action_server = WaypointNavActionServer()
    multi_exec = MultiThreadedExecutor(num_threads=8)
    multi_exec.add_node(waypoint_nav_action_server)
    multi_exec.spin()

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_nav_action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
