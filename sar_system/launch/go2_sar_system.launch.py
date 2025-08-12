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
    Go2 SAR (Search and Rescue) System Launch File
    Launches Unitree Go2 robot in Gazebo world with CHAMP quadruped control
    This creates the shared Gazebo world that drone can connect to later
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
    
    declare_go2_robot_name = DeclareLaunchArgument(
        "go2_robot_name",
        default_value="go2",
        description="Unitree Go2 robot name"
    )
    
    declare_go2_world_init_x = DeclareLaunchArgument(
        "go2_world_init_x", 
        default_value="0.0"
    )
    declare_go2_world_init_y = DeclareLaunchArgument(
        "go2_world_init_y", 
        default_value="-2.0"
    )
    declare_go2_world_init_z = DeclareLaunchArgument(
        "go2_world_init_z", 
        default_value="1.0"
    )
    declare_go2_world_init_heading = DeclareLaunchArgument(
        "go2_world_init_heading", 
        default_value="0.0"
    )

    # ============================================================================
    # ENVIRONMENT SETUP
    # ============================================================================
    
    # PX4 directory path
    px4_dir = '/home/user/shared_volume/PX4-Autopilot'
    
    # Set Gazebo resource paths for PX4 models (shared world for drone later)
    set_gazebo_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        value=f'{px4_dir}/Tools/simulation/gz/models:{px4_dir}/Tools/simulation/gz/worlds'
    )
    
    # World file path  
    world_file = f'{px4_dir}/Tools/simulation/gz/worlds/default.sdf'

    # ============================================================================
    # STEP 1: SHARED GAZEBO WORLD LAUNCH
    # ============================================================================
    
    # Launch Gazebo with PX4 world (shared for both robots)
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
    # STEP 1.5: ROS-GAZEBO BRIDGE 
    # ============================================================================
    
    # Bridge ROS 2 topics to Gazebo Sim (essential for proper simulation)
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration("use_sim_time")},
            {'qos_overrides./clock.publisher.durability': 'transient_local'}
        ],
        arguments=[
            # Gazebo to ROS - Essential for both robots
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            
            # Unitree Go2 specific topics
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # ROS to Gazebo - Control topics
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
    )

    # ============================================================================
    # STEP 2: GO2 ROBOT SETUP
    # ============================================================================
    
    # Go2 package paths and configs
    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    unitree_go2_description = get_package_share_directory("unitree_go2_description")
    
    # Go2 configuration files
    go2_joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    go2_ros_control_config = os.path.join(unitree_go2_sim, "config/ros_control/ros_control.yaml")
    go2_gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    go2_links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    go2_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    
    # Go2 Robot Description
    go2_robot_description = {
        "robot_description": Command(['xacro ', go2_model_path])
    }
    
    # Go2 Robot State Publisher
    go2_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            go2_robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )
    
    # Go2 Spawn Entity with TF setup
    go2_spawn_entity = TimerAction(
        period=3.0,  # 3-second delay to ensure Gazebo is ready
        actions=[
            # Go2 static frame connection (map -> go2/base_link)
            Node(
                package='tf2_ros',
                name='map_to_go2_base_link_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '-2.0', '1.0', '0', '0', '0', 'map', 'go2/base_link'],
            ),
            
            # Go2 URDF connection (go2/base_link -> base_link)
            Node(
                package='tf2_ros',
                name='go2_base_link_to_urdf_base_link_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'go2/base_link', 'base_link'],
            ),
            
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', LaunchConfiguration('go2_robot_name'),
                    '-topic', 'robot_description',
                    '-x', LaunchConfiguration('go2_world_init_x'),
                    '-y', LaunchConfiguration('go2_world_init_y'), 
                    '-z', LaunchConfiguration('go2_world_init_z'),
                    '-Y', LaunchConfiguration('go2_world_init_heading')
                ],
            )
        ]
    )
    
    # Go2 CHAMP Quadruped Controller
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
    
    # Go2 ROS2 Control Spawners
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
    
    # Go2 EKF Localization Nodes
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

    # Go2 TF Transforms completion marker
    go2_tf_transforms = TimerAction(
        period=4.0,  # 1 second after Go2 spawn to ensure TFs are ready
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Go2 SAR System launched - Gazebo world ready for drone connection'"],
                output='screen',
            )
        ]
    )

    # ============================================================================
    # STEP 3: VISUALIZATION (OPTIONAL)
    # ============================================================================
    
    # RViz for Go2 visualization
    sar_system_share = get_package_share_directory("sar_system")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='go2_sar_system_rviz',
        arguments=['-d', os.path.join(sar_system_share, "rviz/sar.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # ============================================================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        declare_go2_robot_name,
        declare_go2_world_init_x,
        declare_go2_world_init_y,
        declare_go2_world_init_z,
        declare_go2_world_init_heading,
        
        # Environment setup
        set_gazebo_resource_path,
        
        # Step 1: Shared Gazebo world (ready for drone connection)
        gz_sim,
        
        # Step 1.5: ROS-Gazebo bridge
        gazebo_bridge,
        
        # Step 2: Go2 Robot with full functionality
        go2_robot_state_publisher,
        go2_spawn_entity,
        go2_quadruped_controller,
        go2_state_estimator,
        go2_controller_spawner_js,
        go2_controller_spawner_effort,
        go2_controller_status_check,
        go2_base_to_footprint_ekf,
        go2_footprint_to_odom_ekf,
        go2_tf_transforms,
        
        # Step 3: Visualization
        rviz_node,
    ])