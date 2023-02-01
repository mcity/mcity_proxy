import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

import time
import math


class Cmd_Vel_Spin(Node):
    """
    Prevents the RMP from self-disabling if nothing is actively publishing on the cmd_vel topic
    """
    def __init__(self):
        super().__init__("cmd_vel_spin")

        self.declare_parameter("timeout_period", 0.1)
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber_ = self.create_subscription(
            Twist, "cmd_vel", self.subscriber_callback, 10
        )
        timer_period = self.get_parameter("timeout_period").value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_angular = Vector3(x=0.0, y=0.0, z=0.0)

    def subscriber_callback(self, msg):
        self.last_linear = msg.linear
        self.last_angular = msg.angular
        self.timer.reset()

    def timer_callback(self):
        if self.last_linear.x <= 0.1:
            self.last_linear.x == 0
        if self.last_linear.y <= 0.1:
            self.last_linear.y == 0
        msg = Twist(
            linear=Vector3(
                x=self.last_linear.x * 0.95,
                y=0.0,
                z=0.0,
            ),
            angular=Vector3(
                x=self.last_angular.z * 0.95,
                y=0.0,
                z=0.0,
            ),
        )
        self.publisher_.publish(msg)


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
