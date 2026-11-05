#!/usr/bin/env python3
# Drives the robot to the last goal clicked in RViz ("2D Goal Pose" -> /goal_pose),
# or through a list of waypoints (nav_msgs/Path on /waypoints, e.g. a mission).
# Turn to face each goal, drive to it; at the last one, turn to its heading. A waypoint at the
# same spot as the one before is a turn in place to its heading (a mission's spin).
# With a laser (/scan): something in the way -> turn toward the open side, drive past it,
# re-aim. Not a planner: no map, no memory of obstacles. Stand-in for Nav2.

import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

MAX_LINEAR = 0.22    # Burger top speed, m/s
MAX_ANGULAR = 1.5    # rad/s (Burger max is 2.84)
POS_TOLERANCE = 0.05   # m
YAW_TOLERANCE = 0.05   # rad
FACING_TOLERANCE = 0.3   # rad; drive forward only when roughly facing the goal
FRONT_HALF_ANGLE = 25    # deg each side of straight ahead that must be clear to drive
SIDE_END_ANGLE = 90      # sides: from the front fan to 90° left / right
BLOCKED = 0.35           # m; closer than this ahead (and before the goal) = blocked
CLEAR = 0.45             # m; turning away stops once the front is this clear
AVOID_FORWARD = 0.4      # m driven past the obstacle before aiming at the goal again
AVOID_SPEED = 0.12       # m/s
AVOID_GIVE_UP = 40.0     # s of dodging before dropping the goal


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.goals = []   # (x, y, yaw, turn) in odom, next one first; turn: turn to yaw in place
        self.pose = None
        # Closest obstacle ahead / left / right (m); inf = nothing seen or no laser
        self.front = self.left = self.right = math.inf
        # Dodging an obstacle: {'turn': +1 left / -1 right, 'from': pose when driving off, 'since'}
        self.avoid = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Gazebo's bridge takes TwistStamped on /cmd_vel, the fake node takes Twist.
        self.stamped = self.declare_parameter('stamped', False).value
        self.cmd_pub = self.create_publisher(
            TwistStamped if self.stamped else Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Path, '/waypoints', self.on_path, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.step)

    def to_odom(self, msg):
        # Goals clicked on the map arrive in "map"; we steer in "odom".
        msg.header.stamp = Time().to_msg()   # latest transform
        try:
            msg = self.tf_buffer.transform(msg, 'odom', timeout=Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(f'Cannot use goal in frame "{msg.header.frame_id}": {e}')
            return None
        p = msg.pose.position
        return (p.x, p.y, yaw_of(msg.pose.orientation))

    def on_goal(self, msg):
        goal = self.to_odom(msg)
        if goal is not None:
            self.goals = [goal + (False,)]
            self.avoid = None
            self.get_logger().info(f'New goal: x={goal[0]:.2f} y={goal[1]:.2f}')

    def on_path(self, msg):
        goals = []
        for pose in msg.poses:
            # Poses in a Path often leave their own frame empty: use the path's
            pose.header.frame_id = pose.header.frame_id or msg.header.frame_id
            goal = self.to_odom(pose)
            if goal is None:
                return
            turn = bool(goals) and math.hypot(goal[0] - goals[-1][0], goal[1] - goals[-1][1]) < 1e-3
            goals.append(goal + (turn,))
        self.goals = goals
        self.avoid = None
        self.get_logger().info(f'New path: {len(goals)} waypoints')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def on_scan(self, msg):
        def closest(beams):
            real = [d for d in beams if not math.isinf(d) and d != 0.0]
            return min(real) if real else math.inf
        # Index 0 = straight ahead on TurtleBot lidars; angles -> indexes with the scan's step
        fan = round(math.radians(FRONT_HALF_ANGLE) / msg.angle_increment)
        side = round(math.radians(SIDE_END_ANGLE) / msg.angle_increment)
        ranges = list(msg.ranges)
        self.front = closest(ranges[:fan + 1] + ranges[-fan:])
        self.left = closest(ranges[fan + 1:side + 1])
        self.right = closest(ranges[-side:-fan])

    def dodge(self, cmd, x, y):
        """Steps around an obstacle. Returns False when done (go back to the goal)."""
        if time.monotonic() - self.avoid['since'] > AVOID_GIVE_UP:
            self.get_logger().warn('Path blocked: giving up on the goal')
            self.goals = []
            self.avoid = None
            return True
        if self.avoid['from'] is None:
            # Turn away until the front is clear, then drive off from here
            if self.front < CLEAR:
                cmd.angular.z = MAX_ANGULAR / 2 * self.avoid['turn']
            else:
                self.avoid['from'] = (x, y)
            return True
        if self.front < BLOCKED:
            self.avoid['from'] = None   # blocked again: keep turning the same way
            return True
        if math.hypot(x - self.avoid['from'][0], y - self.avoid['from'][1]) < AVOID_FORWARD:
            cmd.linear.x = AVOID_SPEED
            return True
        self.avoid = None
        return False

    def step(self):
        if not self.goals or self.pose is None:
            return
        x, y, yaw = self.pose
        gx, gy, gyaw, turn = self.goals[0]
        last = len(self.goals) == 1
        dist = math.hypot(gx - x, gy - y)
        cmd = Twist()

        if dist > POS_TOLERANCE and self.avoid is not None and self.dodge(cmd, x, y):
            pass
        elif dist > POS_TOLERANCE:
            heading_error = wrap(math.atan2(gy - y, gx - x) - yaw)
            cmd.angular.z = clamp(2.0 * heading_error, MAX_ANGULAR)
            if abs(heading_error) < FACING_TOLERANCE:
                # Only obstacles before the goal count: a goal next to a wall stays reachable
                if self.front < BLOCKED and self.front < dist:
                    cmd.angular.z = 0.0
                    self.avoid = {'turn': 1 if self.left > self.right else -1,
                                  'from': None, 'since': time.monotonic()}
                    self.get_logger().info('Obstacle ahead: going around')
                else:
                    # Slow down as things get close ahead
                    cmd.linear.x = min(MAX_LINEAR, dist,
                                       max(AVOID_SPEED / 2, self.front - BLOCKED))
        elif not last and not turn:
            # Waypoint on the way: no need to turn to its heading
            self.goals.pop(0)
            self.get_logger().info(f'Waypoint reached, {len(self.goals)} to go')
        else:
            yaw_error = wrap(gyaw - yaw)
            if abs(yaw_error) > YAW_TOLERANCE:
                cmd.angular.z = clamp(2.0 * yaw_error, MAX_ANGULAR)
            elif not last:
                # one step of a turn in place: a quarter turn, so wrap() keeps the direction
                self.goals.pop(0)
            else:
                self.goals = []
                self.get_logger().info('Goal reached')

        if self.stamped:
            stamped = TwistStamped(twist=cmd)
            stamped.header.stamp = self.get_clock().now().to_msg()
            cmd = stamped
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(WaypointFollower())


if __name__ == '__main__':
    main()
