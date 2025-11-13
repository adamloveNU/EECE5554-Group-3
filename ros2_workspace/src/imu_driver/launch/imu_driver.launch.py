#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for IMU (e.g., /dev/ttyUSB0, /dev/ttyAMA0, /dev/serial0)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data'
    )
    
    # Get launch configuration
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    frame_id = LaunchConfiguration('frame_id')
    
    # IMU driver node
    imu_driver = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate,
            'frame_id': frame_id
        }]
    )
    
    # Optional: Visualizer node
    imu_visualizer = Node(
        package='imu_driver',
        executable='imu_visualizer',
        name='imu_visualizer',
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        imu_driver,
        imu_visualizer,
    ])