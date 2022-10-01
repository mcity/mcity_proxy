import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.srv import OpenLoopControllerDistance, OpenLoopControllerTime

import time
import math


class LinearVelocityOpenLoopController(Node):
    """
    Listens for service calls of the form OpenLoopControlXxxxxx.srv, and executes drive maneuvers accordingly
    """

    def __init__(self):
        super().__init__("linear_velocity_open_loop_controller")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.lv_time_subscriber_ = self.create_service(
            OpenLoopControllerTime, "OpenLoopControllerTime", self.lv_time_callback
        )
        self.lv_distance_subscriber_ = self.create_service(
            OpenLoopControllerDistance,
            "OpenLoopControllerDistance",
            self.lv_distance_callback,
        )
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/odometry/wheel", self.odom_callback, 10
        )
        self.latest_odom = Odometry()

    def lv_time_callback(self, request, response):
        twist = Twist()
        twist.linear.x = float(request.meters_per_second)
        start_time = time.time()
        stopwatch = start_time

        while stopwatch < start_time + float(request.seconds):
            self.publisher_.publish(twist)
            stopwatch = time.time()
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        response.success = True
        return response

    def lv_distance_callback(self, request, response):
        twist = Twist()
        twist.linear.x = float(request.meters_per_second)
        first_odom = self.latest_odom
        print(first_odom)
        print(request.meters)
        while self.distance(self.latest_odom, first_odom) < float(request.meters):
            self.publisher_.publish(twist)
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        response.success = True
        return response

    def odom_callback(self, msg):
        self.latest_odom = msg

    def distance(self, odom1, odom2):
        dist = math.sqrt(
            (float(odom1.pose.pose.position.x) - float(odom2.pose.pose.position.x)) ** 2
            + (float(odom1.pose.pose.position.y) - float(odom2.pose.pose.position.y)) ** 2
        )
        print(dist)
        return dist


def main(args=None):
    rclpy.init(args=args)

    lin_vec_open_loop_control = LinearVelocityOpenLoopController()

    rclpy.spin(lin_vec_open_loop_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lin_vec_open_loop_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
