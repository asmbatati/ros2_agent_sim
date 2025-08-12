#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Master SAR (Search and Rescue) System Launch File
    Launches Go2 and Drone systems with proper timing:
    1. Go2 system (immediate) - includes Gazebo world
    2. Drone system (5-second delay) - connects to existing Gazebo
    """
    
    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )
    
    declare_gui = DeclareLaunchArgument(
        "gui", 
        default_value="true", 
        description="Launch Gazebo with GUI"
    )
    
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true", 
        description="Launch RViz visualization"
    )

    # ============================================================================
    # STEP 1: GO2 SAR SYSTEM (IMMEDIATE)
    # ============================================================================
    
    # Launch Go2 system immediately (includes Gazebo world setup)
    go2_sar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sar_system'),
                'launch',
                'go2_sar_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # ============================================================================
    # STEP 2: DRONE SAR SYSTEM (5-SECOND DELAY)
    # ============================================================================
    
    # Launch drone system with 5-second delay (connects to existing Gazebo)
    drone_sar_delayed_launch = TimerAction(
        period=5.0,  # 5-second delay as requested
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('sar_system'),
                        'launch',
                        'drone_sar_system.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    # ============================================================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        
        # Step 1: Go2 SAR system (immediate - includes Gazebo world)
        go2_sar_launch,
        
        # Step 2: Drone SAR system (5-second delay - connects to existing Gazebo)
        drone_sar_delayed_launch,
    ])