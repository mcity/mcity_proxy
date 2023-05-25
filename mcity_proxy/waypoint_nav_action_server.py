import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import WaypointNav
from mcity_proxy_msgs.msg import Heading
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPoint
from rcl_interfaces.srv import GetParameters

import time
import math
import copy
import numpy as np
from threading import Thread

COURSE_CORRECT_THRESHOLD = 0.01
COURSE_CORRECTION_CUTOFF = 0.25
ANGULAR_ERROR_ABORT = 2 * np.pi / 3
GOAL_POSITION_TOLERANCE = 1.0
KP = 0.5
# KI = 0.004
KI = 0.0
KD = 3.0


class WaypointNavActionServer(Node):
    def __init__(self):
        super().__init__("waypoint_nav_action_server")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.goal_position = np.array([0.0, 0.0])
        self.latest_position = np.array([0.0, 0.0])
        self.latest_heading = 0.0
        self.heading_correction = 0.12339477811 # Default to Estimated Magnetic Declination
        self.kP = 0.5
        self.callback_group = ReentrantCallbackGroup()
        self.param_client = self.create_client(GetParameters,
                                         '/proxy_params/get_parameters')
        request = GetParameters.Request()
        request.names = ['heading_correction']
        self.param_client.wait_for_service()
        future = self.param_client.call_async(request)
        future.add_done_callback(self.callback_heading_correction)

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
        # self.heading_subscriber = self.create_subscription(
        #     Heading,
        #     "/heading",
        #     self.heading_callback,
        #     10,
        #     callback_group=self.callback_group,
        # )
        # self.odom_subscriber = self.create_subscription(
        #     Odometry,
        #     "/odometry/global",
        #     self.odom_callback,
        #     10,
        #     callback_group=self.callback_group,
        # )
        self.fix_subscriber = self.create_subscription(
            NavSatFix,
            "/fix",
            self.fix_callback,
            10,
            callback_group=self.callback_group,
        )
        self.imu_subscriber = self.create_subscription(
            Imu,
            "/imu/filter/data",
            self.imu_callback,
            10,
            callback_group=self.callback_group
        )


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def callback_heading_correction(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("Couldn't get Heading Correction parameter")
        else:
            self.get_logger().info(result.values[0])
            self.heading_correction = result.values[0]



    # def heading_callback(self, msg):
    #     self.latest_heading = float(msg.heading)

    # def odom_callback(self, msg):
    #     self.latest_heading = float(
    #         euler_from_quaternion(
    #             [
    #                 msg.pose.pose.orientation.x,
    #                 msg.pose.pose.orientation.y,
    #                 msg.pose.pose.orientation.z,
    #                 msg.pose.pose.orientation.w,
    #             ]
    #         )[2]
    #     )

    def imu_callback(self, msg):
        self.latest_heading = float(
            euler_from_quaternion(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )[2]
        )

    def fix_callback(self, msg):
        self.latest_position = np.array([msg.latitude, msg.longitude])

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
            if self.latest_position[0] == 0.0 and self.latest_position[1] == 0.0:
                self._goal_handle.abort()
                return
            result = self.navigate_to_target(pt)
            if result is not None:
                return result

        # *Stop ourselves
        twist = Twist()
        self.publisher_.publish(twist)

        self._goal_handle.succeed()
        result = WaypointNav.Result()
        result.final_position = GeoPoint(
            latitude=self.latest_position[0], longitude=self.latest_position[1]
        )
        return result

    def navigate_to_target(self, waypoint):
        self.goal_position = np.array(
            [waypoint.lat_long.latitude, waypoint.lat_long.longitude]
        )

        # * Capture our initial state and extrapolate goal information
        twist = Twist(linear=Vector3(x=waypoint.velocity))
        self.publisher_.publish(twist)
        feedback_msg = WaypointNav.Feedback()

        tick = time.time()
        last_error = 0.0
        integral = 0.0
        dt = time.time()
        curr_dist = self.distance(self.latest_position, self.goal_position)
        # last_dist = 0.0
        while (
            curr_dist
            > GOAL_POSITION_TOLERANCE * waypoint.velocity #and curr_dist >= last_dist
        ):  # *Loop until we've arrived or past our target
            # *Make sure we haven't been cancelled or aborted
            result = self.check_goal_state_change(self._goal_handle)
            if result is not None:
                return result

            # *Update Current Position and Orientation
            goal_bearing = self.calc_bearing(
                self.latest_position,
                self.goal_position,
            )

            # *PID Controller
            error = float(goal_bearing - self.latest_heading - self.heading_correction)
            while error > np.pi:
                error -= 2.0 * np.pi
            while error < -1.0 * np.pi:
                error += 2.0 * np.pi
            # dt = time.time() - dt
            # integral += error * dt
            # derivative = (error - last_error) / dt
            # theta = KP * error + KI * integral + KD * derivative
            # last_error = error
            # twist.angular.z = float(theta)
            twist.angular.z = float(error)
            self.publisher_.publish(twist)

            if time.time() - tick > 1.0:  # * Send feedback every 1 second
                self.get_logger().info("Feedback")
                feedback_msg.current_position = GeoPoint(
                    latitude=self.latest_position[0], longitude=self.latest_position[1]
                )
                feedback_msg.distance_to_goal = self.distance(
                    self.latest_position, self.goal_position
                )
                self._goal_handle.publish_feedback(feedback_msg)
                tick = time.time()
            # last_dist = curr_dist
            curr_dist = self.distance(self.latest_position, self.goal_position)
        # * end while

        return None

    def calc_bearing(self, p1, p2):
        # *Convert latitude and longitude to radians
        lat1 = math.radians(p1[0])
        long1 = math.radians(p1[1])
        lat2 = math.radians(p2[0])
        long2 = math.radians(p2[1])

        # *Calculate the bearing in Radians NED
        bearing = np.arctan2(
            np.sin(long2 - long1) * np.cos(lat2),
            np.cos(lat1) * np.sin(lat2)
            - np.sin(lat1) * np.cos(lat2) * np.cos(long2 - long1),
        )

        enu_bearing = (-1 * bearing) + (
            np.pi / 2.0
        )  # *Return in ENU i.e. positive is counter-clockwise, 0 is east

        while enu_bearing > 2.0 * np.pi:
            enu_bearing -= 2.0 * np.pi
        while enu_bearing < 0.0:
            enu_bearing += 2.0 * np.pi

        return enu_bearing

    def distance(self, p1, p2):
        """
        * Returns the distance between two (lat, long) points, measured in meters
        """
        lat1 = math.radians(p1[0])
        long1 = math.radians(p1[1])
        lat2 = math.radians(p2[0])
        long2 = math.radians(p2[1])
        dLat = lat2 - lat1
        dLon = long2 - long1
        a = np.sin(dLat / 2) * np.sin(dLat / 2) + np.cos(lat1) * np.cos(lat2) * np.sin(
            dLon / 2
        ) * np.sin(dLon / 2)
        return np.abs(12756274 * np.arctan2(np.sqrt(a), np.sqrt(1 - a)))

    def check_goal_state_change(self, goal_handle):
        result = None
        if not goal_handle.is_active:
            # *Another goal was set: abort, and return the current position
            self.get_logger().info("Goal aborted.")
            result = WaypointNav.Result()
            result.final_position = GeoPoint(
                latitude=self.latest_position[0], longitude=self.latest_position[1]
            )
        if goal_handle.is_cancel_requested:
            # *Goal was canceled, stop the robot and return the current position
            self.get_logger().info("Goal cancelled.")
            goal_handle.canceled()
            twist = Twist()
            self.publisher_.publish(twist)
            result = WaypointNav.Result()
            result.final_position = GeoPoint(
                latitude=self.latest_position[0], longitude=self.latest_position[1]
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
