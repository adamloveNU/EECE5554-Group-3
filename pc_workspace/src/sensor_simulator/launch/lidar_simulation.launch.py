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
    default_lidar_file = os.path.join(sensor_sim_share, 'data', 'lidar_data.txt')
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/tmp/vserial3',
        description='Serial port for LiDAR simulator'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    data_file_arg = DeclareLaunchArgument(
        'data_file',
        default_value=default_lidar_file,
        description='Path to LiDAR data file (empty for synthetic mode)'
    )
    
    synthetic_mode_arg = DeclareLaunchArgument(
        'synthetic_mode',
        default_value='false',
        description='Use synthetic data generation (true) or file playback (false)'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier (for file mode)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop playback (for file mode)'
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
    
    scan_rate_arg = DeclareLaunchArgument(
        'scan_rate',
        default_value='10.0',
        description='LiDAR scan rate (Hz)'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    data_file = LaunchConfiguration('data_file')
    synthetic_mode = LaunchConfiguration('synthetic_mode')
    playback_speed = LaunchConfiguration('playback_speed')
    loop = LaunchConfiguration('loop')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    angle_increment = LaunchConfiguration('angle_increment')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')
    scan_rate = LaunchConfiguration('scan_rate')
    
    # Cleanup
    try:
        subprocess.run(['pkill', '-f', 'socat.*pty.*pty'], 
                      stderr=subprocess.DEVNULL)
    except:
        pass
    
    # Create virtual serial ports for LiDAR
    create_virtual_ports = ExecuteProcess(
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
             'sleep 1 && chmod 666 /tmp/vserial3 /tmp/vserial4'],
        name='set_lidar_permissions',
        output='screen'
    )
    
    # LiDAR simulator node
    lidar_simulator = Node(
        package='sensor_simulator',
        executable='lidar_simulator',
        name='lidar_simulator',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': baudrate,
            'data_file': data_file,
            'synthetic_mode': synthetic_mode,
            'playback_speed': playback_speed,
            'loop': loop,
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'range_min': range_min,
            'range_max': range_max,
            'scan_rate': scan_rate
        }]
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        data_file_arg,
        synthetic_mode_arg,
        playback_speed_arg,
        loop_arg,
        angle_min_arg,
        angle_max_arg,
        angle_increment_arg,
        range_min_arg,
        range_max_arg,
        scan_rate_arg,
        create_virtual_ports,
        set_permissions,
        lidar_simulator,
    ])
