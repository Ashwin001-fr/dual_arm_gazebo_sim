#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # 1. Spawn cube (first)
    spawn_cube = Node(
        package='dual_arm_bringup',
        executable='spawn_cube',
        name='spawn_cube',
        output='screen'
    )
    
    # 2. Robot A pick and place (after cube spawns - 2 sec delay)
    robot_a_pick = Node(
        package='dual_arm_bringup',
        executable='robot_a_pick_place',
        name='robot_a_pick_place',
        output='screen'
    )
    
    # 3. Object tracker (after robot_a starts - 2 sec delay)
    object_tracker = Node(
        package='dual_arm_bringup',
        executable='object_tracker',
        name='object_tracker',
        output='screen'
    )
    
    # 4. Motion pattern generator (after tracker - 2 sec delay)
    motion_pattern = Node(
        package='dual_arm_bringup',
        executable='motion_pattern_generator',
        name='motion_pattern_generator',
        output='screen'
    )
    
    # 5. Franka color tracker (after motion pattern - 2 sec delay)
    franka_tracker = Node(
        package='dual_arm_bringup',
        executable='franka_color_tracker',
        name='franka_color_tracker',
        output='screen'
    )
    
    return LaunchDescription([
        # Start with spawn_cube
        spawn_cube,
        
        # Wait 2 seconds, then launch robot_a_pick_place
        TimerAction(
            period=2.0,
            actions=[robot_a_pick]
        ),
        
        # Wait 4 seconds total, then launch object_tracker
        TimerAction(
            period=4.0,
            actions=[object_tracker]
        ),
        
        # Wait 6 seconds total, then launch motion_pattern_generator
        TimerAction(
            period=16.0,
            actions=[motion_pattern]
        ),
        
        # Wait 8 seconds total, then launch franka_color_tracker
        TimerAction(
            period=17.0,
            actions=[franka_tracker]
        ),
    ])
