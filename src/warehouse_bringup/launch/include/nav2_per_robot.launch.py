from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='robot1'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),

        Node(package='nav2_map_server', executable='map_server', namespace=ns,
             name='map_server', parameters=[{'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', namespace=ns,
             name='amcl', parameters=[params_file]),
        Node(package='nav2_controller', executable='controller_server', namespace=ns,
             name='controller_server', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', namespace=ns,
             name='planner_server', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', namespace=ns,
             name='smoother_server', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', namespace=ns,
             name='bt_navigator', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', namespace=ns,
             name='waypoint_follower', parameters=[params_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=ns,
             name='lifecycle_manager', parameters=[{'use_sim_time': True,
                                                    'autostart': True,
                                                    'node_names': [
                                                      'map_server','amcl',
                                                      'controller_server','planner_server',
                                                      'smoother_server','bt_navigator',
                                                      'waypoint_follower'
                                                    ]}]),
    ])
