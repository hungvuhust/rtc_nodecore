#!/usr/bin/env python3

"""
RTC AGV System Launch File

Khởi động tất cả các nodes trong hệ thống RTC AGV:
- Master Node: Quản lý shared memory và publish state tổng
- Navigation Node: Quản lý điều hướng và position
- Battery Node: Monitor battery và charging
- Safety Node: Monitor safety và emergency stop

Autor: RTC Technology JSC
License: Apache 2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # Master Node - khởi động đầu tiên để tạo shared memory
    master_node = Node(
        package='rtc_nodecore',
        executable='rtc_master_node',
        name='rtc_master_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )

    # Navigation Node - khởi động sau master 1 giây
    navigation_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='rtc_nodecore',
                executable='rtc_navigation_node',
                name='rtc_navigation_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )

    # Battery Node - khởi động sau master 1.5 giây
    battery_node = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='rtc_nodecore',
                executable='rtc_battery_node',
                name='rtc_battery_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )

    # Safety Node - khởi động sau master 2 giây
    safety_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rtc_nodecore',
                executable='rtc_safety_node',
                name='rtc_safety_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,

        # Start nodes in sequence
        master_node,
        navigation_node,
        battery_node,
        safety_node,

    ])


def LaunchConfigurationEquals(variable_name, expected_value):
    """Helper function for conditional launching"""
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration

    return IfCondition(
        LaunchConfiguration(variable_name, default=expected_value)
    )
