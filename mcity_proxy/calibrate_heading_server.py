import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
# from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import MoveDistance
from mcity_proxy_msgs.msg import Heading
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPoint
from rcl_interfaces.srv import GetParameters
from mcity_proxy_msgs.srv import CalibrateHeading

import time
import math
import copy
import numpy as np
from threading import Thread, Event

class CalibrateHeadingServer(Node):
    def __init__(self):
        super().__init__("calibrate_heading_server")
        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(CalibrateHeading, 'calibrate_heading', self.calibrate_heading_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.reset()
        self.latest_gps_heading = 0.0
        self.latest_imu_heading = 0.0
        self.latest_position = np.array([0.0, 0.0])

        self.heading_subscriber = self.create_subscription(
            Heading,
            "/heading",
            self.heading_callback,
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
        self.fix_subscriber = self.create_subscription(
            NavSatFix,
            "/fix",
            self.fix_callback,
            10,
            callback_group=self.callback_group,
        )

    def reset(self):
        self.confirmation = False
        self.finished = False

    def heading_callback(self, msg):
        self.latest_gps_heading = float(msg.heading)

    def imu_callback(self, msg):
        self.latest_imu_heading = float(
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

    def calibrate_heading_callback(self, request, response):
        if not self.confirmation:
            self.get_logger().warn("THE PROXY WILL MOVE FORWARD 5 METERS DURING CALIBRATION. CALL SERVICE AGAIN TO CONFIRM PATH IS CLEAR.")
            self.confirmation = True
            response.success = False
            return response

        target_heading = self.latest_imu_heading
        starting_position = self.latest_position

        msg = Twist()
        msg.linear.x = 0.5

        start = time.time()
        while time.time() - start < 10.0:
            msg.angular.z = target_heading - self.latest_imu_heading
            self.cmd_vel_pub.publish(msg)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        time.sleep(1.0)
        correction_factor = self.calc_bearing(starting_position, self.latest_position) - self.latest_imu_heading
        self.get_logger().info(f"Correction Factor: {correction_factor}")
        response.success = True
        msg.linear.x = -0.5

        start = time.time()
        while time.time() - start < 10.0:
            msg.angular.z = target_heading - self.latest_imu_heading
            self.cmd_vel_pub.publish(msg)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.reset()
        return response


def main(args=None):
    rclpy.init(args=args)

    calibrate_heading_server = CalibrateHeadingServer()

    rclpy.spin(calibrate_heading_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibrate_heading_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

