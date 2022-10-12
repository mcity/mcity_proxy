import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import threading

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from mcity_proxy_msgs.action import MoveDistance
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion

import time
import math
import copy
import numpy as np
from threading import Thread


class MoveDistanceActionServer(Node):
    def __init__(self):
        super().__init__("move_distance_action_server")
        self._action_server = ActionServer(
            self,
            MoveDistance,
            "move_distance",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.latest_pose = None
        self.kP = 0.5

    def destroy(self):
        self._action_server.destory()
        super().destroy_node()

    def execute_callback(self, goal_handle):
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

        twist = Twist()
        twist.linear.x = velocity
        while self.latest_pose is None:
            self.get_logger().info("Waiting for starting odometry...")
            time.sleep(1)
        start_pose = copy.deepcopy(self.latest_pose)
        start_position = np.array([start_pose.position.x, start_pose.position.y, 0.0])
        start_heading = self.quaternion_to_euler(start_pose.orientation)
        goal_direction = self.get_direction(start_heading)
        goal_position = start_position + np.dot(distance, goal_direction)
        feedback_msg = MoveDistance.Feedback()
        self.get_logger().info(f"START POSITION: {start_position}")
        self.get_logger().info(f"GOAL POSITION: {goal_position}")

        tick = time.time()
        while math.dist(
            [self.latest_pose.position.x, self.latest_pose.position.y, 0],
            start_position.tolist(),
        ) < np.abs(distance):
            if not goal_handle.is_active:
                # Another goal was set: abort, and return the current position
                self.get_logger().info("Goal aborted.")
                result = MoveDistance.Result()
                result.final_position = Point(
                    x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
                )
                return result
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
            current_position = [
                self.latest_pose.position.x,
                self.latest_pose.position.y,
                0.0,
            ]
            goal_heading = goal_position - np.array(current_position)
            goal_heading_unit = (goal_heading / np.linalg.norm(goal_heading))
            current_heading_unit = self.get_direction(
                self.quaternion_to_euler(self.latest_pose.orientation)
            ) * direction

            distance_remaining = np.abs(distance) - math.dist(
                current_position, start_position.tolist()
            )
            if math.dist(
                current_position,
                start_position.tolist(),
            ) > np.abs(distance * 1.1):
                self.get_logger().info(
                    f"Distance Exceeded: {math.dist([self.latest_pose.position.x, self.latest_pose.position.y, 0], goal_position.tolist())}"
                )
                # Something's gone wrong, stop
                goal_handle.abort()
                twist = Twist()
                self.publisher_.publish(twist)
                result = MoveDistance.Result()
                result.final_position = Point(
                    x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
                )
                return result
            theta = (
                self.kP
                * distance_remaining
                / distance
                * (
                    np.arccos(
                        np.clip(
                            np.dot(goal_heading_unit, current_heading_unit),
                            -1.0,
                            1.0,
                        )
                    )
                ) * direction
            )
            cross_product = np.cross(goal_heading_unit, current_heading_unit)
            if cross_product[2] > 0.0:
                theta *= -1.0
            # elif np.isclose(cross_product[2], 0.0):
            #     theta = 0.0
            # if np.abs(theta) > np.pi / 2:
            #     self.get_logger().info(
            #         f"Angle Exceeded: {math.dist(current_position, goal_position.tolist())}, {theta}"
            #     )
            #     # Something's gone wrong, stop
            #     goal_handle.abort()
            #     twist = Twist()
            #     self.publisher_.publish(twist)
            #     result = MoveDistance.Result()
            #     result.final_position = Point(
            #         x=self.latest_pose.position.x,
            #         y=self.latest_pose.position.y,
            #         z=0.0,
            #     )
            #     return result
            twist.angular.z = theta
            self.publisher_.publish(twist)

            if time.time() - tick > 1.0:
                feedback_msg.current_position = Point(
                    x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
                )
                goal_handle.publish_feedback(feedback_msg)
                tick = time.time()

        twist = Twist()
        self.publisher_.publish(twist)

        goal_handle.succeed()
        result = MoveDistance.Result()
        result.final_position = Point(
            x=self.latest_pose.position.x, y=self.latest_pose.position.y, z=0.0
        )
        return result

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

    def quaternion_to_euler(self, q):
        euler = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
        heading = euler / np.linalg.norm(euler)
        if any(np.isnan(heading)):
            heading = np.array([0.0, 0.0, 0.0])
        return heading

    def get_direction(self, euler):
        pitch = euler[1]
        yaw = euler[2]
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return np.array([x, y, z])


class OdomUpdater(Node):
    """
    Updates Odometry on an Action Server
    """

    def __init__(self, server):
        super().__init__("odom_updater")
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        self.server = server

    def odom_callback(self, msg):
        self.server.latest_pose = copy.deepcopy(msg.pose.pose)


def spin_odom_updater(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)

    move_distance_action_server = MoveDistanceActionServer()
    multi_exec = MultiThreadedExecutor()

    odom_updater = OdomUpdater(move_distance_action_server)
    odom_updater_exec = SingleThreadedExecutor()
    odom_updater_exec.add_node(odom_updater)
    odom_updater_thread = Thread(
        target=spin_odom_updater, args=(odom_updater_exec,), daemon=True
    )
    odom_updater_thread.start()

    rclpy.spin(move_distance_action_server, executor=multi_exec)

    # Destroy nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_distance_action_server.destroy()
    odom_updater.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
