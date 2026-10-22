# Fake TurtleBot3 Burger in RViz, driven by clicking goals ("2D Goal Pose").
# Same nodes as turtlebot3_fake_node.launch.py, plus our waypoint follower
# and our own RViz config.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    fake_node_params = os.path.join(
        get_package_share_directory('turtlebot3_fake_node'), 'param', MODEL + '.yaml')
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_' + MODEL + '.urdf')

    return LaunchDescription([
        Node(
            package='turtlebot3_fake_node',
            executable='turtlebot3_fake_node',
            parameters=[fake_node_params]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf]),
        ExecuteProcess(
            cmd=['python3', os.path.join(ROOT, 'scripts', 'waypoint_follower.py')],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(ROOT, 'config', 'turtlebot.rviz')]),
    ])
