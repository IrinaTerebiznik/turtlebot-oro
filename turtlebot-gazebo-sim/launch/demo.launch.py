# Fake TurtleBot3 Burger on the maps/map.yaml floor plan, driven by clicking goals ("2D Goal Pose").
# Same nodes as turtlebot3_fake_node.launch.py, plus our waypoint follower
# and our own RViz config.
# gazebo:=true swaps the fake robot for the Gazebo one (turtlebot3_world, with a simulated laser).
# map:=true serves maps/map.yaml and places the robot on GO; otherwise map = odom, no map.
# slam:=true builds the map live with slam_toolbox (needs a laser: gazebo or real; not with map:=true),
# and drives with Nav2 (config/nav2.yaml): it plans around obstacles on that map. Routes on /waypoints
# go through scripts/nav2_bridge.py, goals (/goal_pose) straight to Nav2. Without SLAM there is no
# map for Nav2, so scripts/waypoint_follower.py drives instead (straight lines, dodging).
# real:=true drives a real TurtleBot3 over the network: it runs its own bringup (motors, laser, IMU,
# robot_state_publisher), so no fake or Gazebo robot here.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    fake_node_params = os.path.join(
        get_package_share_directory('turtlebot3_fake_node'), 'param', MODEL + '.yaml')
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_' + MODEL + '.urdf')
    gazebo_world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    # Gazebo runs its own clock: everything else follows it when it's on.
    gazebo = LaunchConfiguration('gazebo')
    real = LaunchConfiguration('real')
    # Gazebo and the real robot (ROS 2 Jazzy) take TwistStamped on /cmd_vel; the fake node, Twist
    stamped = PythonExpression(
        ["'true' if '", gazebo, "' == 'true' or '", real, "' == 'true' else 'false'"])
    fake = PythonExpression(["'", gazebo, "' != 'true' and '", real, "' != 'true'"])
    use_map = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    no_slam = PythonExpression(["'", slam, "' != 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument('gazebo', default_value='false'),
        DeclareLaunchArgument('map', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('real', default_value='false'),
        Node(
            package='turtlebot3_fake_node',
            executable='turtlebot3_fake_node',
            parameters=[fake_node_params],
            condition=IfCondition(fake)),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf],
            condition=IfCondition(fake)),
        # Gazebo + robot + its own robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_world),
            condition=IfCondition(gazebo)),
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{'yaml_filename': os.path.join(ROOT, 'maps', 'map.yaml'),
                         'use_sim_time': gazebo}],
            condition=IfCondition(use_map)),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
            condition=IfCondition(use_map)),
        # Robot starts on GO (bottom-right square), facing Mediterranean Avenue.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '9.0', '--y', '0.73', '--yaw', '3.14159',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            condition=IfCondition(use_map)),
        # No map: keep a "map" frame (RViz and the agent use it), same as odom.
        # Not with SLAM: it publishes map -> odom itself.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            condition=IfCondition(PythonExpression(
                ["'", use_map, "' != 'true' and '", slam, "' != 'true'"]))),
        # SLAM: /scan + odom -> /map and map -> odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'use_sim_time': gazebo}.items(),
            condition=IfCondition(slam)),
        # Waypoint list for ORO's Waypoints actions (local/waypoints.yaml)
        ExecuteProcess(
            cmd=['python3', os.path.join(ROOT, 'scripts', 'waypoint_manager.py'),
                 '--ros-args', '-p', ['use_sim_time:=', gazebo]],
            output='screen'),
        # Robot data -> ORO key-values (agent/cac/DataSourceDefinition.yaml)
        ExecuteProcess(
            cmd=['python3', os.path.join(ROOT, 'scripts', 'oro_kv_bridge.py'),
                 '--ros-args', '-p', ['stamped:=', stamped]],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', os.path.join(ROOT, 'scripts', 'waypoint_follower.py'),
                 '--ros-args', '-p', ['use_sim_time:=', gazebo], '-p', ['stamped:=', stamped]],
            output='screen',
            condition=IfCondition(no_slam)),
        # With SLAM: Nav2 on the live map, and the bridge that runs /waypoints routes through it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'use_sim_time': gazebo, 'autostart': 'true',
                              'params_file': os.path.join(ROOT, 'config', 'nav2.yaml')}.items(),
            condition=IfCondition(slam)),
        ExecuteProcess(
            cmd=['python3', os.path.join(ROOT, 'scripts', 'nav2_bridge.py'),
                 '--ros-args', '-p', ['use_sim_time:=', gazebo]],
            output='screen',
            condition=IfCondition(slam)),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(ROOT, 'config', 'turtlebot.rviz')],
            parameters=[{'use_sim_time': gazebo}]),
    ])
