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
    rviz_config = os.path.join(slam_viz_share, 'rviz', 'imu_visualization.rviz')
    plotjuggler_config = os.path.join(slam_viz_share, 'plotjuggler', 'imu_layout.xml')
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    use_plotjuggler_arg = DeclareLaunchArgument(
        'use_plotjuggler',
        default_value='true',
        description='Launch PlotJuggler'
    )
    
    load_layout_arg = DeclareLaunchArgument(
        'load_layout',
        default_value='true',
        description='Load PlotJuggler layout'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_plotjuggler = LaunchConfiguration('use_plotjuggler')
    load_layout = LaunchConfiguration('load_layout')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
# PlotJuggler node WITH layout and auto-start
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=[
            '--layout', plotjuggler_config,
            '--start_streamer', 'ROS2 Topic Subscriber'
        ],
        output='screen',
        condition=IfCondition(use_plotjuggler)
    )
    
    return LaunchDescription([
        use_rviz_arg,
        use_plotjuggler_arg,
        load_layout_arg,
        rviz_node,
        plotjuggler_node,
    ])