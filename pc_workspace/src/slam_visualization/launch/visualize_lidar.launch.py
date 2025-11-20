#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Get paths
    slam_viz_share = get_package_share_directory('slam_visualization')
    rviz_config = os.path.join(slam_viz_share, 'rviz', 'lidar_visualization.rviz')
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        use_rviz_arg,
        rviz_node,
    ])