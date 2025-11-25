import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #Get the paths
    pkg_share = get_package_share_directory('description')
    newbot_urdf_path = os.path.join(pkg_share, 'urdf', 'tricycle.urdf')
    obstacle_urdf_path = os.path.join(pkg_share, 'urdf', 'obstacle.urdf')

    #Read Tricycle Descritpion
    with open(newbot_urdf_path, 'r') as file:
        robot_description_content = file.read()
        
    return LaunchDescription([
        #Include Gazeboo Launch File
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
        ),

        #Robot State Publisher For Tricycle
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_tricycle', 
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        #Spawn Tricycle from topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_tricycle',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'tricycle', '-x', '0', '-y', '0', '-z', '0.1'],

        ),

        #Spawn obstacles directly from file
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_box',
            output='screen',
            arguments=['-file', obstacle_urdf_path, '-entity', 'obstacles', '-x', '0', '-y', '0', '-z', '0.0'],
        ),

        # Node to get cmdvel
        Node(
            package='description',
            executable='exe',
            name='print_val',
            output='screen',
        ),

        Node(
            package='description',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen',
        )
    ])