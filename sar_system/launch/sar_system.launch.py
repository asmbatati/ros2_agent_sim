#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    SAR (Search and Rescue) System Launch File - Step 2 Fixed
    Combines Unitree Go2 and Drone simulation in single Gazebo world
    Uses minimal namespacing following original working patterns
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

    # ============================================================================
    # ENVIRONMENT SETUP
    # ============================================================================
    
    # PX4 directory path
    px4_dir = '/home/user/shared_volume/PX4-Autopilot'
    
    # Set Gazebo resource paths for PX4 models
    set_gazebo_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        value=f'{px4_dir}/Tools/simulation/gz/models:{px4_dir}/Tools/simulation/gz/worlds'
    )
    
    # World file path  
    world_file = f'{px4_dir}/Tools/simulation/gz/worlds/default.sdf'

    # ============================================================================
    # STEP 1: SINGLE GAZEBO WORLD LAUNCH
    # ============================================================================
    
    # Launch Gazebo with PX4 world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': f'{world_file} -r',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ============================================================================
    # STEP 1.5: ROS-GAZEBO BRIDGE (CRITICAL FOR CLOCK SYNC)
    # ============================================================================
    
    # Bridge ROS 2 topics to Gazebo Sim (essential for proper simulation)
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        arguments=[
            # Clock synchronization (CRITICAL)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states from Gazebo to ROS
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            # TF from Gazebo to ROS  
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
    )

    # ============================================================================
    # STEP 2: GO2 ROBOT SPAWN (IMMEDIATE - 0 SECONDS)
    # ============================================================================
    
    # Go2 package paths and configs (using original patterns)
    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    unitree_go2_description = get_package_share_directory("unitree_go2_description")
    
    # Go2 configuration files (original files, no modification needed)
    go2_joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    go2_ros_control_config = os.path.join(unitree_go2_sim, "config/ros_control/ros_control.yaml")
    go2_gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    go2_links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    go2_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    
    # Go2 Robot Description (GLOBAL - required for gazebo_ros2_control)
    go2_robot_description = {
        "robot_description": Command(['xacro ', go2_model_path])
    }
    
    # Go2 Robot State Publisher (GLOBAL namespace - following original pattern)
    go2_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            go2_robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )
    
    # Go2 Spawn Entity in Gazebo (different entity name for identification)
    go2_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'go2',
            '-topic', 'robot_description',
            '-x', '5.0',
            '-y', '0.0', 
            '-z', '0.0'
        ],
    )
    
    # Go2 CHAMP Quadruped Controller (minimal namespace - only for cmd_vel)
    go2_quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(['xacro ', go2_model_path])},
            go2_joints_config,
            go2_links_config,
            go2_gait_config,
            {"hardware_connected": False},
            {"close_loop_odom": True},
        ],
        remappings=[
            ("/cmd_vel/smooth", "/go2/cmd_vel"),
        ],
    )

    # Go2 CHAMP State Estimator 
    go2_state_estimator = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"orientation_from_imu": True},
            {"urdf": Command(['xacro ', go2_model_path])},
            go2_joints_config,
            go2_links_config,
            go2_gait_config,
        ],
    )
    
    # Go2 ROS2 Control Spawners (GLOBAL namespace - following original pattern)
    go2_controller_spawner_js = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_states_controller",
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            )
        ]
    )

    go2_controller_spawner_effort = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_group_effort_controller",
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            )
        ]
    )
    
    # Go2 EKF Localization Nodes (with go2 namespace for odom topics)
    go2_base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="go2_base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config", "ekf", "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "/go2/odom/local")],
    )

    go2_footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node", 
        name="go2_footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config", "ekf", "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "/go2/odom")],
    )
    
    # Go2 Controller Status Check 
    go2_controller_status_check = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Go2 Controller Status:' && ros2 control list_controllers"],
                output='screen',
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
        
        # Environment setup
        set_gazebo_resource_path,
        
        # Step 1: Single Gazebo world
        gz_sim,
        
        # Step 1.5: ROS-Gazebo bridge (CRITICAL for clock sync)
        gazebo_bridge,
        
        # Step 2: Go2 Robot (immediate spawn) - minimal namespace approach
        go2_robot_state_publisher,
        go2_spawn_entity,
        go2_quadruped_controller,
        go2_state_estimator,
        go2_controller_spawner_js,
        go2_controller_spawner_effort,
        go2_controller_status_check,
        go2_base_to_footprint_ekf,
        go2_footprint_to_odom_ekf,
        
        # TODO: Step 3 - Drone (60 second delay) will be added next
    ])