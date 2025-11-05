#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    comport_arg = DeclareLaunchArgument(
        'comport',
        default_value='/dev/ttyUSB0',
        description='Serial port device path for the Leica total station'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Node configuration
    leica_serial_node = Node(
        package='leica_serial_ros2',
        executable='leica_serial_node',
        name='leica_serial_node',
        output='screen',
        parameters=[{
            'comport': LaunchConfiguration('comport'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        comport_arg,
        log_level_arg,
        leica_serial_node,
    ])