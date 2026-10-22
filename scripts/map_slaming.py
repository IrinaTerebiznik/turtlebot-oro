#!/usr/bin/env python3
# Drives the robot around SLAMing to create a map
# Optional: stay within a radius (m) of the start point, stop after a duration (s), e.g.
#   python3 map_slaming.py --ros-args -p radius:=3.0 -p duration:=180


import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time

# Look at a fan of beams, not just beam 0: one beam is thinner than the robot.
# 20° each side of straight ahead covers its width.
# Angles are turned into indexes with the scan's own step (1° on the sim, about 1° on real
# TurtleBot lidars); index 0 = straight ahead on TurtleBot lidars (angle_min = 0).
FRONT_HALF_ANGLE = 20
SIDE_END_ANGLE = 90      # sides = from the edge of the front fan to 90° (left) / 270° (right)
STOP_DISTANCE = 0.5      # m; closer than this ahead = turn
FORWARD_SPEED = 0.1      # m/s
TURN_SPEED = 0.5         # rad/s
RETURN_INSIDE = 0.8      # back outside the radius: head home until within 80% of it
FACING_TOLERANCE = 0.3   # rad; head home forward only when roughly facing it


def closest(beams):
    # Start at infinity: any real reading is closer. No readings at all = path clear.
    min_distance = math.inf
    for distance in beams:
        # Keep only real readings: not inf AND not 0.0
        if not math.isinf(distance) and distance != 0.0:
            min_distance = distance if distance < min_distance else min_distance
    return min_distance


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MapSlammer(Node):
    def __init__(self):
        super().__init__('map_slammer')
        # +1 = turning left, -1 = right, None = driving forward
        self.turn_direction = None
        # 0 = no limit. Measured in odom from where the robot was when this node started.
        self.radius = self.number_param('radius')
        self.start = None
        # 0 = run until stopped
        self.duration = self.number_param('duration')
        self.started_at = time.monotonic()
        self.pose = None
        self.returning = False
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def number_param(self, name):
        # Accept radius:=2 as well as radius:=2.0 (ROS types them differently)
        any_type = ParameterDescriptor(dynamic_typing=True)
        return float(self.declare_parameter(name, 0.0, any_type).value)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        if self.start is None:
            self.start = (p.x, p.y)

    def home_turn(self):
        """With a radius: -1/+1 to turn back toward the start, 0 to go straight, None = no limit hit."""
        if not self.radius or self.pose is None:
            return None
        x, y, yaw = self.pose
        sx, sy = self.start
        dist = math.hypot(x - sx, y - sy)
        if dist > self.radius:
            self.returning = True
        elif dist < self.radius * RETURN_INSIDE:
            self.returning = False
        if not self.returning:
            return None
        error = wrap(math.atan2(sy - y, sx - x) - yaw)
        if abs(error) < FACING_TOLERANCE:
            return 0
        return 1 if error > 0 else -1

    def on_scan(self, msg):
        if self.duration and time.monotonic() - self.started_at > self.duration:
            self.get_logger().info(f'Done: explored for {self.duration:.0f} s')
            raise SystemExit
        ranges = list(msg.ranges)
        # Number of beams that cover a given angle
        fan = round(math.radians(FRONT_HALF_ANGLE) / msg.angle_increment)
        side = round(math.radians(SIDE_END_ANGLE) / msg.angle_increment)
        # Front wraps around the list: left = start (0..20°), right = end (340..359°)
        front = ranges[:fan + 1] + ranges[-fan:]
        # Left grows from the front fan up to 90°, right from 270° up to the front fan
        left = ranges[fan + 1:side + 1]
        right = ranges[-side:-fan]

        min_distance = closest(front)
        if min_distance >= STOP_DISTANCE:
            self.turn_direction = None
        elif self.turn_direction is None:
            # Blocked: turn toward the side whose nearest obstacle is farther away.
            # Keep that side until the front is clear, or it wiggles in corners.
            self.turn_direction = 1 if closest(left) > closest(right) else -1

        # Obstacles first; with the path clear, the radius may steer us home.
        home = self.home_turn() if self.turn_direction is None else None
        turn = self.turn_direction if home is None else home

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        if not turn:
            cmd.twist.linear.x = FORWARD_SPEED
        else:
            cmd.twist.angular.z = TURN_SPEED * turn
        self.cmd_pub.publish(cmd)

        status = {None: 'forward', 0: 'forward', 1: 'turn left', -1: 'turn right'}[turn]
        if home is not None:
            status += ' (back inside radius)'
        self.get_logger().info(f'Closest object: {min_distance:.2f} m, status: {status}')


def main():
    rclpy.init()
    node = MapSlammer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Don't leave the robot rolling on its last command
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        node.cmd_pub.publish(stop)


if __name__ == '__main__':
    main()
