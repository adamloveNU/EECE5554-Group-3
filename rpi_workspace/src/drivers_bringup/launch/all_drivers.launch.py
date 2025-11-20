#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get path to robot description launch file
    robot_desc_share = get_package_share_directory('robot_description')
    robot_state_publisher_launch = os.path.join(
        robot_desc_share, 'launch', 'robot_state_publisher.launch.py'
    )
    
    # Include robot state publisher (publishes TF)
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_state_publisher_launch)
    )
    
    # IMU Driver
    imu_node = Node(
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
    
    # Camera Driver
    camera_node = Node(
        package='camera_driver',
        executable='pi_camera_node',
        name='pi_camera_node',
        output='screen',
        parameters=[{
            'pipe_path': '/tmp/camera_pipe',
            'frame_rate': 30.0,
            'frame_id': 'camera_link',
            'camera_name': 'pi_camera'
        }]
    )
    
    # LiDAR Driver
    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'port': '/tmp/vserial4',
            'baudrate': 115200,
            'frame_id': 'laser_link',
            'scan_topic': 'scan'
        }]
    )
    
    return LaunchDescription([
        robot_description,
        imu_node,
        camera_node,
        lidar_node,
    ])