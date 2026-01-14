#!/usr/bin/env python3
"""
Launch file for MeCO Menu System
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('meco_menu')
    
    # Default config file
    default_config = os.path.join(pkg_dir, 'config', 'menu.yaml')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'menu_config',
        default_value=default_config,
        description='Path to menu configuration YAML file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Run in debug mode (terminal UI)'
    )
    
    # Menu node
    menu_node = Node(
        package='meco_menu',
        executable='menu_node',
        name='meco_menu',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('menu_config')
        }]
    )
    
    # Debug node
    menu_debug_node = Node(
        package='meco_menu',
        executable='menu_node_debug',
        name='meco_menu_debug',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('menu_config')
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('debug'))
    )
    
    return LaunchDescription([
        config_arg,
        debug_arg,
        menu_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
