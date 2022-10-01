import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.srv import OpenLoopControllerDistance, OpenLoopControllerTime

import time
import math
import copy
from threading import Thread


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
        self.start_x = 0.0
        self.start_y = 0.0
        self.latest_x = 0.0
        self.latest_y = 0.0

    def lv_time_callback(self, request, response):
        twist = Twist()
        twist.linear.x = float(request.meters_per_second)
        start_time = time.time()
        stopwatch = start_time

        while stopwatch < start_time + float(request.seconds):
            self.publisher_.publish(twist)
            stopwatch = time.time()
        twist = Twist()
        self.publisher_.publish(twist)
        response.success = True
        return response

    def lv_distance_callback(self, request, response):
        twist = Twist()
        twist.linear.x = float(request.meters_per_second)
        self.start_x = copy.deepcopy(self.latest_x)
        self.start_y = copy.deepcopy(self.latest_y)
        while math.dist(
            (self.start_x, self.start_y), (self.latest_x, self.latest_y)
        ) < float(request.meters):
            self.publisher_.publish(twist)
        twist = Twist()
        self.publisher_.publish(twist)
        response.success = True
        return response

class OdomUpdater(Node):
    """
    Updates Odometry on an instance of LinearVelocityOpenLoopController
    """

    def __init__(self, lin_vel_olc):
        super().__init__("odom_updater")
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/odometry/wheel", self.odom_callback, 10
        )
        self.lin_vel_olc = lin_vel_olc

    def odom_callback(self, msg):
        self.lin_vel_olc.latest_x = copy.deepcopy(msg.pose.pose.position.x)
        self.lin_vel_olc.latest_y = copy.deepcopy(msg.pose.pose.position.y)

def spin_odom_updater(executor):
     try:
          executor.spin()
     except rclpy.executors.ExternalShutdownException:
          pass

def main(args=None):
    rclpy.init(args=args)

    lin_vec_open_loop_control = LinearVelocityOpenLoopController()

    odom_updater = OdomUpdater(lin_vec_open_loop_control)
    odom_updater_exec = SingleThreadedExecutor()
    odom_updater_exec.add_node(odom_updater)
    odom_updater_thread = Thread(target=spin_odom_updater, args=(odom_updater_exec, ), daemon=True)
    odom_updater_thread.start()

    rclpy.spin(lin_vec_open_loop_control)

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lin_vec_open_loop_control.destroy_node()
    odom_updater.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
