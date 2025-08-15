#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare


def spawn_robot(ns: str, x: float, y: float, yaw: float) -> GroupAction:
    """
    Spawn one robot into Gazebo Sim and publish its TF via robot_state_publisher.
    Assumes your Xacro declares a property 'prefix' and supports: xacro ... prefix:=<ns>_
    """
    xacro_path = PathJoinSubstitution(
        [FindPackageShare('warehouse_description'), 'urdf', 'amr.xacro']
    )
    robot_desc = Command([
        FindExecutable(name='xacro'),
        TextSubstitution(text=' '), xacro_path,
        TextSubstitution(text=' '), TextSubstitution(text=f'prefix:={ns}_'),
    ])

    return GroupAction([
        PushRosNamespace(ns),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                # No '-world' flag: targets /world/default (recommended).
                '-name', ns,
                '-string', robot_desc,
                '-x', str(x),
                '-y', str(y),
                '-Y', str(yaw),
            ],
        ),
    ])


def generate_launch_description() -> LaunchDescription:
    # World SDF path (make sure <world name="default"> in your SDF)
    world_path = PathJoinSubstitution(
        [FindPackageShare('warehouse_bringup'), 'worlds', 'warehouse.world']
    )

    # Start Gazebo Sim SERVER only (headless) and run immediately: avoids GLIBC GUI issue
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_path],
        output='screen'
    )

    # Delay spawns a bit so /world/default services are available
    robot1 = TimerAction(period=2.0, actions=[spawn_robot('robot1', 1.0, 1.0, 0.0)])
    robot2 = TimerAction(period=2.8, actions=[spawn_robot('robot2', 2.0, 1.0, 0.0)])

    return LaunchDescription([
        gz_server,
        robot1,
        robot2,
    ])
