#!/usr/bin/env python3
import subprocess
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get path to data file
    pkg_share = get_package_share_directory('sensor_simulator')
    default_data_file = os.path.join(pkg_share, 'data', 'imu_data.txt')
    
    # Declare launch arguments
    data_file_arg = DeclareLaunchArgument(
        'data_file',
        default_value=default_data_file,
        description='Path to IMU data file'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier (1.0 = real-time, 0.5 = half speed, 2.0 = double speed)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop playback when reaching end of file'
    )
    
    # Get launch configuration
    data_file = LaunchConfiguration('data_file')
    playback_speed = LaunchConfiguration('playback_speed')
    loop = LaunchConfiguration('loop')
    
    # Cleanup
    try:
        subprocess.run(['pkill', '-f', 'socat.*pty.*pty'], 
                      stderr=subprocess.DEVNULL)
    except:
        pass
    
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
    
    imu_simulator = Node(
        package='sensor_simulator',
        executable='imu_simulator',
        name='imu_simulator',
        output='screen',
        parameters=[{
            'port': '/tmp/vserial1',
            'baudrate': 9600,
            'data_file': data_file,
            'playback_speed': playback_speed,
            'loop': loop
        }]
    )
    
    return LaunchDescription([
        data_file_arg,
        playback_speed_arg,
        loop_arg,
        create_virtual_ports,
        set_permissions,
        imu_simulator,
    ])