# Fake TurtleBot3 Burger on the maps/map.yaml floor plan, driven by clicking goals ("2D Goal Pose").
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
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{'yaml_filename': os.path.join(ROOT, 'maps', 'map.yaml')}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        # Robot starts on GO (bottom-right square), facing Mediterranean Avenue.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '9.0', '--y', '0.73', '--yaw', '3.14159',
                       '--frame-id', 'map', '--child-frame-id', 'odom']),
        ExecuteProcess(
            cmd=['python3', os.path.join(ROOT, 'scripts', 'waypoint_follower.py')],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(ROOT, 'config', 'turtlebot.rviz')]),
    ])
