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
    default_imu_file = os.path.join(sensor_sim_share, 'data', 'imu_data.txt')
    default_lidar_file = os.path.join(sensor_sim_share, 'data', 'lidar_data.txt')
    default_video_file = os.path.join(sensor_sim_share, 'data', 'camera', 'camera_simulator_video.MOV')
    
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
    
    # LiDAR arguments
    lidar_synthetic_mode_arg = DeclareLaunchArgument(
        'lidar_synthetic_mode',
        default_value='false',
        description='Use synthetic LiDAR data generation'
    )
    
    lidar_data_file_arg = DeclareLaunchArgument(
        'lidar_data_file',
        default_value=default_lidar_file,
        description='Path to LiDAR data file (empty for synthetic mode)'
    )
    
    lidar_playback_speed_arg = DeclareLaunchArgument(
        'lidar_playback_speed',
        default_value='1.0',
        description='LiDAR playback speed multiplier'
    )
    
    lidar_loop_arg = DeclareLaunchArgument(
        'lidar_loop',
        default_value='true',
        description='Loop LiDAR playback'
    )
    
    lidar_scan_rate_arg = DeclareLaunchArgument(
        'lidar_scan_rate',
        default_value='10.0',
        description='LiDAR scan rate (Hz)'
    )
    
    # Get launch configurations
    imu_data_file = LaunchConfiguration('imu_data_file')
    imu_playback_speed = LaunchConfiguration('imu_playback_speed')
    imu_loop = LaunchConfiguration('imu_loop')
    
    video_file = LaunchConfiguration('video_file')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_playback_speed = LaunchConfiguration('camera_playback_speed')
    camera_loop = LaunchConfiguration('camera_loop')
    
    lidar_synthetic_mode = LaunchConfiguration('lidar_synthetic_mode')
    lidar_data_file = LaunchConfiguration('lidar_data_file')
    lidar_playback_speed = LaunchConfiguration('lidar_playback_speed')
    lidar_loop = LaunchConfiguration('lidar_loop')
    lidar_scan_rate = LaunchConfiguration('lidar_scan_rate')
    
    # Cleanup
    try:
        subprocess.run(['pkill', '-f', 'socat.*pty.*pty'], 
                      stderr=subprocess.DEVNULL)
    except:
        pass
    
    # Create virtual serial ports for IMU
    create_imu_ports = ExecuteProcess(
        cmd=[
            'socat', '-d', '-d',
            'pty,raw,echo=0,link=/tmp/vserial1',
            'pty,raw,echo=0,link=/tmp/vserial2'
        ],
        name='socat_imu_ports',
        output='log'
    )
    
    # Create virtual serial ports for LiDAR
    create_lidar_ports = ExecuteProcess(
        cmd=[
            'socat', '-d', '-d',
            'pty,raw,echo=0,link=/tmp/vserial3',
            'pty,raw,echo=0,link=/tmp/vserial4'
        ],
        name='socat_lidar_ports',
        output='log'
    )
    
    set_permissions = ExecuteProcess(
        cmd=['bash', '-c', 
             'sleep 1 && chmod 666 /tmp/vserial1 /tmp/vserial2 /tmp/vserial3 /tmp/vserial4'],
        name='set_permissions',
        output='screen'
    )
    
    # IMU Simulator
    imu_simulator = Node(
        package='sensor_simulator',
        executable='imu_simulator',
        name='imu_simulator',
        output='screen',
        parameters=[{
            'port': '/tmp/vserial1',
            'baudrate': 9600,
            'data_file': imu_data_file,
            'playback_speed': imu_playback_speed,
            'loop': imu_loop
        }]
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
    
    # LiDAR Simulator
    lidar_simulator = Node(
        package='sensor_simulator',
        executable='lidar_simulator',
        name='lidar_simulator',
        output='screen',
        parameters=[{
            'port': '/tmp/vserial3',
            'baudrate': 115200,
            'data_file': lidar_data_file,
            'synthetic_mode': lidar_synthetic_mode,
            'playback_speed': lidar_playback_speed,
            'loop': lidar_loop,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,  # ~1 degree
            'range_min': 0.1,
            'range_max': 10.0,
            'scan_rate': lidar_scan_rate
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
        # LiDAR arguments
        lidar_synthetic_mode_arg,
        lidar_data_file_arg,
        lidar_playback_speed_arg,
        lidar_loop_arg,
        lidar_scan_rate_arg,
        # Nodes
        create_imu_ports,
        create_lidar_ports,
        set_permissions,
        imu_simulator,
        camera_simulator,
        lidar_simulator,
    ])