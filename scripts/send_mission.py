#!/usr/bin/env python3
# Sends one mission from config/missions.yaml to the waypoint follower (/waypoints) and exits.
#   python3 send_mission.py 2

import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

MISSIONS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'missions.yaml')
WAIT_FOR_FOLLOWER = 5.0   # s


def main():
    with open(MISSIONS) as f:
        config = yaml.safe_load(f)
    missions = config['missions']
    name = sys.argv[1] if len(sys.argv) > 1 else ''
    if name not in missions:
        sys.exit(f'Unknown mission "{name}". Known: {", ".join(missions)}')

    path = Path()
    path.header.frame_id = config['frame']
    for point in missions[name]['waypoints']:
        pose = PoseStamped()
        pose.header.frame_id = config['frame']
        pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
        yaw = float(point[2]) if len(point) > 2 else 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        path.poses.append(pose)

    rclpy.init()
    node = rclpy.create_node('send_mission')
    pub = node.create_publisher(Path, '/waypoints', 10)
    # A message published before the follower is matched is lost
    deadline = time.monotonic() + WAIT_FOR_FOLLOWER
    while pub.get_subscription_count() == 0:
        if time.monotonic() > deadline:
            sys.exit('No waypoint follower listening on /waypoints')
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(path)
    time.sleep(0.5)   # let it go out before we exit
    print(f'Mission {name} ({missions[name]["label"]}): {len(path.poses)} waypoints sent')


if __name__ == '__main__':
    main()
