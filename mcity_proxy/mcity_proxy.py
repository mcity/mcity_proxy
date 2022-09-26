import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mcity_proxy_msgs.srv import LinearVelocityFromTime, LinearVelocityFromDistance

import time
import math

class Cmd_Vel_Spin(Node):
    """
    Prevents the RMP from self-disabling if nothing is actively publishing on the cmd_vel topic
    """

    def __init__(self):
        super().__init__("cmd_vel_spin")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber_ = self.create_subscription(
            Twist, "cmd_vel", self.subscriber_callback, 10
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def subscriber_callback(self, msg):
        self.timer.reset()

    def timer_callback(self):
        msg = Twist()
        self.publisher_.publish(msg)

class Linear_Controller(Node):
    """
    Listens for service calls of the form LinearVelocityXXX.srv, and executes drive maneuvers accordingly
    """

    def __init__(self):
        super().__init__("linear_controller")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.lv_time_subscriber_ = self.create_subscription(
            LinearVelocityFromTime, "topic", self.lv_time_callback, 10
        )
        self.lv_distance_subscriber_ = self.create_subscription(
            LinearVelocityFromDistance, "topic", self.lv_distance_callback, 10
        )
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        self.latest_odom = Odometry()

    def lv_time_callback(self, request, response):
        twist = Twist()
        twist.linear.x = float(request.meters_per_second)
        time = time.time()

        while(time < time + float(request.seconds)):
            self.publisher_.publish(twist)
            time -= 1
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        response = True
        return response

    def lv_distance_callback(self, request, response):
        twist = Twist()
        twist.linear.x = float(request.meters_per_second)
        first_odom = self.latest_odom
        while(self.distance(self.latest_odom, first_odom) < float(request.meters)):
            self.publisher_.publish(twist)
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        response = True
        return response

    def odometry_callback(self, msg):
        self.latest_odom = msg

    def distance(odom1, odom2):
        return math.sqrt((odom1.pose.pose.position.x - odom2.pose.pose.position.x)**2 + (odom1.pose.pose.position.y - odom2.pose.pose.position.y)**2)


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_spin = Cmd_Vel_Spin()

    rclpy.spin(cmd_vel_spin)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_spin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
