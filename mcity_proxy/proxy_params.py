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

import time
import math
import copy
import numpy as np
from threading import Thread

class ProxyParams(Node):
    def __init__(self):
        super().__init__("proxy_params", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

def main(args=None):
    rclpy.init(args=args)

    proxy_params = ProxyParams()

    rclpy.spin(proxy_params)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    proxy_params.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
