#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('description')
    tricycle_urdf = os.path.join(pkg_share, 'urdf', 'tricycle.urdf')
    obstacle_urdf = os.path.join(pkg_share, 'urdf', 'obstacle.urdf')

    with open(tricycle_urdf, 'r') as f:
        robot_description_content = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_tricycle',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, {'use_sim_time': True}],
    )

    spawn_tricycle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tricycle',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'tricycle', '-x', '0', '-y', '0', '-z', '0.1'],
    )

    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_box',
        output='screen',
        arguments=['-file', obstacle_urdf, '-entity', 'obstacles', '-x', '0.8', '-y', '0', '-z', '0.0'],
    )

    # Your driver/teleop; force it to publish /cmd_vel_raw
    driver = Node(
        package='description',
        executable='exe',       # your existing driver node
        name='driver',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_raw')],  # remap driver output
    )

    # Camera-based safety filter: /cmd_vel_raw -> /cmd_vel (zeros if <= 0.5 m)
    safety = Node(
        package='description',
        executable='camera_sub',
        name='camera_safety',
        output='screen',
        parameters=[{
            'stop_distance': 0.5,
            'region_ratio': 0.33,
            'depth_topic': '/camera/depth/image_raw',
            'cmd_in': '/cmd_vel_raw',
            'cmd_out': '/cmd_vel',
        }],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        TimerAction(period=2.0, actions=[spawn_tricycle]),
        TimerAction(period=2.5, actions=[spawn_obstacle]),
        driver,
        safety,
    ])
