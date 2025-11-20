#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'slam_robot.urdf.xacro'
    )
    
    # Process xacro to get URDF
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Optional: Joint state publisher (for non-fixed joints, not needed for now)
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    # )
    
    return LaunchDescription([
        robot_state_publisher,
    ])