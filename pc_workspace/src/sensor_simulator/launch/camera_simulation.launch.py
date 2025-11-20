#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get paths
    sensor_sim_share = get_package_share_directory('sensor_simulator')
    default_video_file = os.path.join(sensor_sim_share, 'data', 'camera', 'camera_simulator_video.MOV')
    
    # Declare launch arguments
    video_file_arg = DeclareLaunchArgument(
        'video_file',
        default_value=default_video_file,
        description='Path to video file'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',  # ✅ This stays float
        description='Camera frame rate (fps)'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop video playback'
    )
    
    pipe_path_arg = DeclareLaunchArgument(
        'pipe_path',
        default_value='/tmp/camera_pipe',
        description='Path to camera pipe'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    # Get launch configurations
    video_file = LaunchConfiguration('video_file')
    fps = LaunchConfiguration('fps')
    playback_speed = LaunchConfiguration('playback_speed')
    loop = LaunchConfiguration('loop')
    pipe_path = LaunchConfiguration('pipe_path')
    frame_id = LaunchConfiguration('frame_id')
    
    # Camera simulator node
    camera_simulator = Node(
        package='sensor_simulator',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[{
            'video_file': video_file,
            'fps': fps,
            'playback_speed': playback_speed,
            'loop': loop,
            'pipe_path': pipe_path
        }]
    )
    
    return LaunchDescription([
        video_file_arg,
        fps_arg,
        playback_speed_arg,
        loop_arg,
        pipe_path_arg,
        frame_id_arg,
        camera_simulator,
    ])