#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    comport_arg = DeclareLaunchArgument(
        'comport',
        default_value='/dev/ttyUSB0',
        description='Serial port device path for the Leica total station'
    )

    # Main Leica serial node
    leica_serial_node = Node(
        package='leica_serial_ros2',
        executable='leica_serial_node',
        name='leica_serial_node',
        output='screen',
        parameters=[{
            'comport': LaunchConfiguration('comport'),
        }],
        respawn=True,
        respawn_delay=2.0
    )

    # Optional: RViz2 for visualization (uncomment if you want to visualize the position)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', '/path/to/your/rviz/config.rviz']  # Update with your RViz config
    # )

    # Optional: Static transform publisher for visualization context
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        output='screen'
    )

    # Delayed start for static transform (give the main node time to start)
    delayed_static_tf = TimerAction(
        period=2.0,
        actions=[static_tf_node]
    )

    return LaunchDescription([
        comport_arg,
        leica_serial_node,
        delayed_static_tf,
        # rviz_node,  # Uncomment if you want RViz
    ])