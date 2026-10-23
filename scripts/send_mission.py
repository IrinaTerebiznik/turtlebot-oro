#!/usr/bin/env python3
# Sends one mission from config/missions.yaml to the waypoint follower (/waypoints) and exits.
#   python3 send_mission.py 2
# A {spin: N} entry becomes 4 * N waypoints at the previous waypoint's position, each a quarter
# turn further left from the heading it arrived on: the follower turns in place through them.

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


def expand(waypoints):
    """missions.yaml entries -> [(x, y, yaw)], spins expanded into turn-in-place waypoints."""
    points = []
    for entry in waypoints:
        if isinstance(entry, dict):
            if not points:
                sys.exit('A spin needs a waypoint before it')
            x, y, _ = points[-1]
            # arrival heading: from the waypoint before (or the start, map origin) to this one
            px, py = points[-2][:2] if len(points) > 1 else (0.0, 0.0)
            yaw = math.atan2(y - py, x - px)
            for _ in range(4 * int(entry['spin'])):
                yaw += math.pi / 2
                points.append((x, y, yaw))
        else:
            points.append((float(entry[0]), float(entry[1]), float(entry[2]) if len(entry) > 2 else 0.0))
    return points


def main():
    with open(MISSIONS) as f:
        config = yaml.safe_load(f)
    missions = config['missions']
    name = sys.argv[1] if len(sys.argv) > 1 else ''
    if name not in missions:
        sys.exit(f'Unknown mission "{name}". Known: {", ".join(missions)}')

    path = Path()
    path.header.frame_id = config['frame']
    for x, y, yaw in expand(missions[name]['waypoints']):
        pose = PoseStamped()
        pose.header.frame_id = config['frame']
        pose.pose.position.x, pose.pose.position.y = x, y
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
