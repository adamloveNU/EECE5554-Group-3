#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    pipe_path_arg = DeclareLaunchArgument(
        'pipe_path',
        default_value='/tmp/camera_pipe',
        description='Path to camera pipe (FIFO)'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='Camera frame rate (fps)'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='pi_camera',
        description='Camera name'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    # Get launch configurations
    pipe_path = LaunchConfiguration('pipe_path')
    frame_rate = LaunchConfiguration('frame_rate')
    camera_name = LaunchConfiguration('camera_name')
    frame_id = LaunchConfiguration('frame_id')
    
    # Camera driver node
    camera_driver = Node(
        package='camera_driver',
        executable='pi_camera_node',
        name='pi_camera_node',
        output='screen',
        parameters=[{
            'pipe_path': pipe_path,
            'frame_rate': frame_rate,
            'camera_name': camera_name,
            'frame_id': frame_id
        }]
    )
    
    return LaunchDescription([
        pipe_path_arg,
        frame_rate_arg,
        camera_name_arg,
        frame_id_arg,
        camera_driver,
    ])

