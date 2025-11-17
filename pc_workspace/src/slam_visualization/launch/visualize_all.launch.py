#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Get paths
    slam_viz_share = get_package_share_directory('slam_visualization')
    rviz_config = os.path.join(slam_viz_share, 'rviz', 'slam_full.rviz')
    plotjuggler_config = os.path.join(slam_viz_share, 'plotjuggler', 'slam_full_layout.xml')
    
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
    
    plotjuggler_delay_arg = DeclareLaunchArgument(
        'plotjuggler_delay',
        default_value='2.0',
        description='Delay before launching PlotJuggler (seconds)'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_plotjuggler = LaunchConfiguration('use_plotjuggler')
    plotjuggler_delay = LaunchConfiguration('plotjuggler_delay')
    
    # RViz node (shows both IMU and Camera)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # PlotJuggler node (for IMU data plotting)
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=['--layout', plotjuggler_config, '--buffer_size', '30'],
        output='screen',
        condition=IfCondition(use_plotjuggler)
    )
    
    # Delay PlotJuggler start
    delayed_plotjuggler = TimerAction(
        period=plotjuggler_delay,
        actions=[plotjuggler_node]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        use_plotjuggler_arg,
        plotjuggler_delay_arg,
        rviz_node,
        delayed_plotjuggler,
    ])