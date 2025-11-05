#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


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
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame ID for the published position'
    )
    
    child_frame_id_arg = DeclareLaunchArgument(
        'child_frame_id',
        default_value='leica_pos',
        description='Child frame ID for the TF transform'
    )

    # Set use_sim_time parameter globally if requested
    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time'),
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('use_sim_time'), '" == "true"'
            ])
        )
    )

    # Main Leica serial node
    leica_serial_node = Node(
        package='leica_serial_ros2',
        executable='leica_serial_node',
        name='leica_serial_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'comport': LaunchConfiguration('comport'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            # Uncomment and modify these if you need different topic names
            # ('/leica/position', '/custom/leica/position'),
            # ('/leica/start_stop', '/custom/leica/start_stop'),
            # ('/paintcopter/position', '/custom/paintcopter/position'),
        ]
    )

    return LaunchDescription([
        comport_arg,
        log_level_arg,
        namespace_arg,
        use_sim_time_arg,
        frame_id_arg,
        child_frame_id_arg,
        set_use_sim_time,
        leica_serial_node,
    ])