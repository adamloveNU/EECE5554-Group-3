#!/usr/bin/env python3
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get paths
    sensor_sim_share = get_package_share_directory('sensor_simulator')
    default_imu_file = os.path.join(sensor_sim_share, 'data', 'sample_imu_data.txt')
    default_video_file = os.path.join(sensor_sim_share, 'data', 'camera', 'test_video.MOV')
    
    # Declare launch arguments
    # IMU arguments
    imu_data_file_arg = DeclareLaunchArgument(
        'imu_data_file',
        default_value=default_imu_file,
        description='Path to IMU data file'
    )
    
    imu_playback_speed_arg = DeclareLaunchArgument(
        'imu_playback_speed',
        default_value='0.5',
        description='IMU playback speed multiplier'
    )
    
    imu_loop_arg = DeclareLaunchArgument(
        'imu_loop',
        default_value='true',
        description='Loop IMU playback'
    )
    
    # Camera arguments
    video_file_arg = DeclareLaunchArgument(
        'video_file',
        default_value=default_video_file,
        description='Path to video file'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30.0', 
        description='Camera frame rate (fps)'
    )
    
    camera_playback_speed_arg = DeclareLaunchArgument(
        'camera_playback_speed',
        default_value='1.0',
        description='Camera playback speed multiplier'
    )
    
    camera_loop_arg = DeclareLaunchArgument(
        'camera_loop',
        default_value='true',
        description='Loop camera playback'
    )
    
    # Get launch configurations
    imu_data_file = LaunchConfiguration('imu_data_file')
    imu_playback_speed = LaunchConfiguration('imu_playback_speed')
    imu_loop = LaunchConfiguration('imu_loop')
    
    video_file = LaunchConfiguration('video_file')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_playback_speed = LaunchConfiguration('camera_playback_speed')
    camera_loop = LaunchConfiguration('camera_loop')
    
    # Cleanup
    try:
        subprocess.run(['pkill', '-f', 'socat.*pty.*pty'], 
                      stderr=subprocess.DEVNULL)
    except:
        pass
    
    # Create virtual serial ports for IMU
    create_virtual_ports = ExecuteProcess(
        cmd=[
            'socat', '-d', '-d',
            'pty,raw,echo=0,link=/tmp/vserial1',
            'pty,raw,echo=0,link=/tmp/vserial2'
        ],
        name='socat_virtual_ports',
        output='log'
    )
    
    set_permissions = ExecuteProcess(
        cmd=['bash', '-c', 
             'sleep 1 && chmod 666 /tmp/vserial1 /tmp/vserial2'],
        name='set_permissions',
        output='screen'
    )
    
    # IMU Simulator
    imu_simulator = Node(
        package='sensor_simulator',
        executable='file_imu_simulator',
        name='file_imu_simulator',
        output='screen',
        parameters=[{
            'port': '/tmp/vserial1',
            'baudrate': 9600,
            'data_file': imu_data_file,
            'playback_speed': imu_playback_speed,
            'loop': imu_loop
        }]
    )
    
    # IMU Driver
    imu_driver = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': '/tmp/vserial2',
            'baudrate': 9600,
            'frame_id': 'imu_link'
        }]
    )
    
    # IMU Visualizer (text output)
    imu_visualizer = Node(
        package='imu_driver',
        executable='imu_visualizer',
        name='imu_visualizer',
        output='screen'
    )
    
    # Camera Simulator
    camera_simulator = Node(
        package='sensor_simulator',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[{
            'video_file': video_file,
            'fps': camera_fps,
            'playback_speed': camera_playback_speed,
            'loop': camera_loop,
            'pipe_path': '/tmp/camera_pipe'
        }]
    )
    
    # Camera Driver
    camera_driver = Node(
        package='camera_driver',
        executable='pi_camera_node',
        name='pi_camera_node',
        output='screen',
        parameters=[{
            'pipe_path': '/tmp/camera_pipe',
            'frame_rate': camera_fps,
            'frame_id': 'camera_link',
            'camera_name': 'pi_camera'
        }]
    )
    
    return LaunchDescription([
        # IMU arguments
        imu_data_file_arg,
        imu_playback_speed_arg,
        imu_loop_arg,
        # Camera arguments
        video_file_arg,
        camera_fps_arg,
        camera_playback_speed_arg,
        camera_loop_arg,
        # Nodes
        create_virtual_ports,
        set_permissions,
        imu_simulator,
        imu_driver,
        imu_visualizer,
        camera_simulator,
        camera_driver,
    ])