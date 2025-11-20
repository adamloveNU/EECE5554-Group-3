#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/tmp/vserial4',
        description='Serial port for LiDAR (e.g., /tmp/vserial4, /dev/ttyUSB0)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_link',
        description='Frame ID for LiDAR data'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Topic name for publishing scan data'
    )
    
    angle_min_arg = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.14159',
        description='Minimum scan angle (radians)'
    )
    
    angle_max_arg = DeclareLaunchArgument(
        'angle_max',
        default_value='3.14159',
        description='Maximum scan angle (radians)'
    )
    
    angle_increment_arg = DeclareLaunchArgument(
        'angle_increment',
        default_value='0.0174533',
        description='Angle increment between measurements (radians, ~1 degree)'
    )
    
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.1',
        description='Minimum range value (meters)'
    )
    
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='10.0',
        description='Maximum range value (meters)'
    )
    
    # Get launch configuration
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    frame_id = LaunchConfiguration('frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    angle_increment = LaunchConfiguration('angle_increment')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    
    # LiDAR driver node
    lidar_driver = Node(
        package='lidar_driver',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate,
            'frame_id': frame_id,
            'scan_topic': scan_topic,
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'range_min': range_min,
            'range_max': range_max,
            'time_increment': 0.0,
            'scan_time': 0.1
        }]
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        scan_topic_arg,
        angle_min_arg,
        angle_max_arg,
        angle_increment_arg,
        range_min_arg,
        range_max_arg,
        lidar_driver,
    ])

