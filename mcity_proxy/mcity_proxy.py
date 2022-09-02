import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


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
