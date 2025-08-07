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
    SAR (Search and Rescue) System Launch File - Complete
    Combines Unitree Go2 and Drone simulation in single Gazebo world
    Go2: Immediate spawn at (0,-1,1) with CHAMP quadruped control
    Drone: 5-second delayed spawn at (0,0,0.3) with PX4 SITL + MAVROS
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
    # STEP 1.5: ROS-GAZEBO BRIDGE 
    # ============================================================================
    
    # Bridge ROS 2 topics to Gazebo Sim (essential for proper simulation)
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        arguments=[
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
            '-x', '0.0',
            '-y', '-2.0', 
            '-z', '1.0'
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
    # STEP 3: DRONE SPAWN (DELAYED - 5 SECONDS)
    # ============================================================================
    
    # Drone integration using original launch pattern with TimerAction delay
    # Following original drone_sim/launch/drone.launch.py structure
    
    drone_delayed_spawn = TimerAction(
        period=5.0,  # 5-second delay as requested
        actions=[
            
            # 3.1: PX4 SITL Process
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'cd /home/user/shared_volume/PX4-Autopilot && ' +
                    'PX4_SYS_AUTOSTART=4021 ' +
                    'PX4_GZ_MODEL=x500_lidar_camera ' +
                    'PX4_UXRCE_DDS_NS=drone ' +
                    'PX4_GZ_MODEL_POSE="0,0,0.3" ' +
                    'PX4_GZ_WORLD=default ' +
                    'PX4_GZ_STANDALONE=1 ' +
                    'PX4_SIM_SPEED_FACTOR=1.0 ' +
                    'PX4_PARAM_SYS_LOGGER=1 ' +               # Reduce logging verbosity
                    'PX4_PARAM_SENS_IMU_MODE=0 ' +            # Disable strict IMU validation
                    './build/px4_sitl_default/bin/px4 -i 1 2>/dev/null'  # Suppress stderr
                ],
                output='screen',
            ),
            
            # 3.2: micro-XRCE-DDS Agent (port 8888)
            ExecuteProcess(
                cmd=[
                    'MicroXRCEAgent', 'udp4', '-p', '8888'
                ],
                output='screen',
            ),
            
            # 3.3: MAVROS with Drone Configuration  
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('drone_sim'),
                        'launch',
                        'mavros.launch.py'
                    ])
                ]),
                launch_arguments={
                    'mavros_namespace': 'drone/mavros',
                    'tgt_system': '2',  # Different system ID from default
                    'fcu_url': 'udp://:14541@127.0.0.1:14557',  # Different ports
                    'pluginlists_yaml': os.path.join(
                        get_package_share_directory('drone_sim'), 
                        'mavros', 'drone_px4_pluginlists.yaml'
                    ),
                    'config_yaml': os.path.join(
                        get_package_share_directory('drone_sim'),
                        'mavros', 'drone_px4_config.yaml'
                    ),
                    'base_link_frame': 'drone/base_link',
                    'odom_frame': 'drone/odom',
                    'map_frame': 'map',
                    'use_sim_time': 'True'
                }.items()
            ),
        ]
    )
    
    # Drone Static TF Transforms (following original pattern)
    # These handle coordinate frame conversions (ENU ↔ NED, map ↔ odom)
    drone_tf_transforms = TimerAction(
        period=65.0,  # 5 seconds after drone spawn
        actions=[
            # Static TF map/world -> local_pose_ENU
            Node(
                package='tf2_ros',
                name='map2px4_drone_tf_node',
                executable='static_transform_publisher',
                arguments=['0.0', '0.0', '0.3', '0.0', '0', '0', 'map', 'drone/odom'],
            ),
            
            # Static TF base_link -> Gimbal_Camera
            Node(
                package='tf2_ros',
                name='drone_base2gimbal_camera_tf_node',
                executable='static_transform_publisher',
                arguments=['0.1', '0', '0.13', '1.5708', '0', '1.5708', 
                          'drone/base_link', 'drone/gimbal_camera'],
            ),
            
            # Static TF base_link -> Lidar
            Node(
                package='tf2_ros',
                name='drone_base2lidar_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '0', '0.295', '0', '0', '0', 
                          'drone/base_link', 'x500_lidar_camera_1/lidar_link/gpu_lidar'],
            ),
            
            # Connect drone/base_link to base_link (for multi-robot)
            Node(
                package='tf2_ros',
                name='drone_base_to_base_link_tf_node', 
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'drone/base_link', 'base_link'],
            ),
            
            # Base link to base_link_frd (ENU to NED conversion)
            Node(
                package='tf2_ros',
                name='base_link_to_frd_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'base_link', 'base_link_frd'],
            ),
        ]
    )
    
    # Drone ROS-Gazebo Bridge for Sensors (following original pattern)
    drone_sensor_bridge = TimerAction(
        period=62.0,  # 2 seconds after drone spawn  
        actions=[
            Node(
                package='ros_gz_bridge',
                name='drone_sensor_bridge',
                executable='parameter_bridge',
                arguments=[
                    # Camera data
                    '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    
                    # Lidar data  
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    
                    # IMU and other sensors
                    '/imu_gimbal@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    # '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                    '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                    
                    # Gimbal control
                    '/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double',
                    '/gimbal/cmd_roll@std_msgs/msg/Float64]gz.msgs.Double',
                    '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
                ],
                # Remap topics to drone namespace (following original pattern)
                remappings=[
                    ('/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image', '/drone/gimbal_camera'),
                    ('/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info', '/drone/gimbal_camera_info'),
                    ('/gimbal/cmd_yaw', '/drone/gimbal/cmd_yaw'),
                    ('/gimbal/cmd_roll', '/drone/gimbal/cmd_roll'),
                    ('/gimbal/cmd_pitch', '/drone/gimbal/cmd_pitch'),
                    ('/imu_gimbal', '/drone/imu_gimbal'),
                    ('/scan', '/drone/scan'),
                    ('/scan/points', '/drone/scan/points'),
                    # ('/world/default/model/x500_lidar_camera_1/link/base_link/sensor/imu_sensor/imu', '/drone/imu'),
                    ('/world/default/model/x500_lidar_camera_1/link/base_link/sensor/air_pressure_sensor/air_pressure', '/drone/air_pressure'),
                    ('/navsat', '/drone/gps'),
                ],
            )
        ]
    )
    
    # Drone Status Check
    drone_status_check = TimerAction(
        period=75.0,  # Check status after all drone components should be loaded
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Drone Status Check:' && ros2 topic list | grep drone && echo 'MAVROS Topics:' && ros2 topic list | grep mavros"],
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
        
        # Step 3: Drone (5-second delayed spawn) - following original pattern
        drone_delayed_spawn,
        drone_tf_transforms,
        drone_sensor_bridge,
        drone_status_check,
    ])