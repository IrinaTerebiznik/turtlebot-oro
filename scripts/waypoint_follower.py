#!/usr/bin/env python3
# Drives the robot to the last goal clicked in RViz ("2D Goal Pose" -> /goal_pose).
# Turn to face the goal, drive to it, then turn to the goal heading.
# Stand-in for Nav2 while we run the fake robot (no laser, so no Nav2).

import math

import rclpy
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

MAX_LINEAR = 0.22    # Burger top speed, m/s
MAX_ANGULAR = 1.5    # rad/s (Burger max is 2.84)
POS_TOLERANCE = 0.05   # m
YAW_TOLERANCE = 0.05   # rad
FACING_TOLERANCE = 0.3   # rad; drive forward only when roughly facing the goal


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.goal = None
        self.pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.05, self.step)

    def on_goal(self, msg):
        # Goals clicked on the map arrive in "map"; we steer in "odom".
        msg.header.stamp = Time().to_msg()   # latest transform
        try:
            msg = self.tf_buffer.transform(msg, 'odom', timeout=Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(f'Cannot use goal in frame "{msg.header.frame_id}": {e}')
            return
        p = msg.pose.position
        self.goal = (p.x, p.y, yaw_of(msg.pose.orientation))
        self.get_logger().info(f'New goal: x={p.x:.2f} y={p.y:.2f}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def step(self):
        if self.goal is None or self.pose is None:
            return
        x, y, yaw = self.pose
        gx, gy, gyaw = self.goal
        dist = math.hypot(gx - x, gy - y)
        cmd = Twist()

        if dist > POS_TOLERANCE:
            heading_error = wrap(math.atan2(gy - y, gx - x) - yaw)
            cmd.angular.z = clamp(2.0 * heading_error, MAX_ANGULAR)
            if abs(heading_error) < FACING_TOLERANCE:
                cmd.linear.x = min(MAX_LINEAR, dist)
        else:
            yaw_error = wrap(gyaw - yaw)
            if abs(yaw_error) > YAW_TOLERANCE:
                cmd.angular.z = clamp(2.0 * yaw_error, MAX_ANGULAR)
            else:
                self.goal = None
                self.get_logger().info('Goal reached')

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(WaypointFollower())


if __name__ == '__main__':
    main()
