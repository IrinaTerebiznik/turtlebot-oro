#!/usr/bin/env python3
# Keeps a list of waypoints, driven by ORO's Waypoints actions (agent/user_scripts/waypoint.sh):
# commands on /waypoint_cmd: add (current pose), remove_last, clear,
# visit (drive all of them in order, starting from the next one, as a route on /waypoints).
# Publishes the list and the next waypoint as ORO key-values. Saved in local/waypoints.yaml.

import json
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

STORE = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'local', 'waypoints.yaml')
FRAME = 'map'
ROBOT_FRAME = 'base_footprint'
ARRIVED = 0.15         # m; this close to the waypoint = visited (Nav2's goal tolerance is 0.1 m)
PUBLISH_PERIOD = 1.0   # s
ARRIVAL_PERIOD = 0.1   # s; the robot drives through tour waypoints without stopping


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        self.waypoints = []   # [x, y, yaw] in map
        self.next = 0         # index of the next one to visit
        self.visiting = []    # indexes still to reach on the current tour, in order
        self.load()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Path, '/waypoints', 10)
        self.kv_pub = self.create_publisher(String, '/inorbit/custom_data', 10)
        self.create_subscription(String, '/waypoint_cmd', self.on_cmd, 10)
        self.create_timer(PUBLISH_PERIOD, self.publish)
        self.create_timer(ARRIVAL_PERIOD, self.check_arrival)

    def load(self):
        if os.path.exists(STORE):
            with open(STORE) as f:
                data = yaml.safe_load(f) or {}
            self.waypoints = data.get('waypoints', [])
            self.next = data.get('next', 0)

    def save(self):
        with open(STORE, 'w') as f:
            yaml.safe_dump({'waypoints': self.waypoints, 'next': self.next}, f)

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(FRAME, ROBOT_FRAME, Time())
        except TransformException as e:
            self.get_logger().warn(f'No robot pose in "{FRAME}": {e}')
            return None
        p = t.transform.translation
        return [round(p.x, 2), round(p.y, 2), round(yaw_of(t.transform.rotation), 2)]

    def on_cmd(self, msg):
        cmd = msg.data.strip()
        if cmd == 'add':
            pose = self.robot_pose()
            if pose is not None:
                self.waypoints.append(pose)
                self.get_logger().info(f'Added waypoint {len(self.waypoints)}: {pose}')
        elif cmd == 'remove_last':
            if self.waypoints:
                self.get_logger().info(f'Removed waypoint {len(self.waypoints)}')
                self.waypoints.pop()
        elif cmd == 'clear':
            self.waypoints = []
            self.get_logger().info('Waypoint list erased')
        elif cmd == 'visit':
            self.visit()
        else:
            self.get_logger().warn(f'Unknown waypoint command "{cmd}"')
            return
        # Keep "next" pointing at a waypoint that still exists
        if self.next >= len(self.waypoints):
            self.next = 0
        if cmd != 'visit':
            # The list changed: a tour in progress no longer matches it
            self.visiting = []
        self.save()
        self.publish()

    def visit(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to visit')
            return
        # Every waypoint once, starting from "next" and wrapping around
        n = len(self.waypoints)
        self.visiting = [(self.next + i) % n for i in range(n)]
        path = Path()
        path.header.frame_id = FRAME
        path.header.stamp = self.get_clock().now().to_msg()
        for index in self.visiting:
            x, y, yaw = self.waypoints[index]
            pose = PoseStamped()
            pose.header.frame_id = FRAME
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(pose)
        self.path_pub.publish(path)
        order = ', '.join(str(i + 1) for i in self.visiting)
        self.get_logger().info(f'Visiting waypoints {order}')

    def check_arrival(self):
        # Reached the current one: the following waypoint becomes "next" (wrapping around)
        if self.visiting:
            pose = self.robot_pose()
            index = self.visiting[0]
            x, y, _ = self.waypoints[index]
            if pose is not None and math.hypot(pose[0] - x, pose[1] - y) < ARRIVED:
                self.visiting.pop(0)
                self.next = (index + 1) % len(self.waypoints)
                self.save()
                left = f', {len(self.visiting)} to go' if self.visiting else ': tour done'
                self.get_logger().info(f'Reached waypoint {index + 1}{left}')

    def publish(self):
        listed = [{'n': i + 1, 'x': w[0], 'y': w[1]} for i, w in enumerate(self.waypoints)]
        if self.waypoints:
            x, y, _ = self.waypoints[self.next]
            next_text = f'{self.next + 1}: ({x}, {y})'
        else:
            next_text = 'none'
        for key, value in (('waypoints', json.dumps(listed)), ('next_waypoint', next_text)):
            self.kv_pub.publish(String(data=f'{key}={value}'))


def main():
    rclpy.init()
    rclpy.spin(WaypointManager())


if __name__ == '__main__':
    main()
