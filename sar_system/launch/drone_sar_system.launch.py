#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Drone SAR (Search and Rescue) System Launch File
    Spawns drone in existing Gazebo world with PX4 SITL + MAVROS
    
    Prerequisites: 
    - go2_sar_system.launch.py must be running (provides Gazebo world)
    - Use this to add drone to existing SAR system
    """
    
    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    # ============================================================================
    # STEP 1: DRONE SPAWN AND SYSTEMS
    # ============================================================================
    
    # Drone integration - connects to existing Gazebo world
    drone_delayed_spawn = TimerAction(
        period=2.0,  # Short delay to ensure connection to existing Gazebo
        actions=[
            
            # 1.1: PX4 SITL Process
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
                    'PX4_PARAM_SYS_LOGGER=0 ' +               # Disable verbose logging
                    'PX4_PARAM_SENS_IMU_MODE=1 ' +            # Enable proper IMU mode
                    'PX4_PARAM_SYS_HAS_GPS=1 ' +              # Enable GPS
                    'PX4_PARAM_EKF2_AID_MASK=1 ' +            # Enable GPS aiding
                    'PX4_PARAM_EKF2_HGT_MODE=0 ' +            # Barometric height mode
                    './build/px4_sitl_default/bin/px4 -i 1'
                ],
                output='screen',
            ),
            
            # 1.2: micro-XRCE-DDS Agent (port 8888) with reduced verbosity
            ExecuteProcess(
                cmd=[
                    'MicroXRCEAgent', 'udp4', '-p', '8888', '-v0'
                ],
                output='screen',
            ),
            
            # 1.3: MAVROS with Drone Configuration  
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
        
    # ============================================================================
    # STEP 2: DRONE TF TRANSFORMS
    # ============================================================================
    
    # Drone Static TF Transforms (EXACT copy from original working code)
    drone_tf_transforms = TimerAction(
        period=4.0,  # After drone spawn completes
        actions=[
            # Map->odom transform (essential for navigation)
            Node(
                package='tf2_ros',
                name='map_to_odom_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            ),
            
            # Odom to odom_ned (ENU to NED conversion)
            Node(
                package='tf2_ros',
                name='odom_to_odom_ned_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'odom', 'odom_ned'],
            ),
            
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
            
            # Connect drone/base_link to base_link (for multi-robot) - FROM ORIGINAL
            Node(
                package='tf2_ros',
                name='drone_base_to_base_link_tf_node', 
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'drone/base_link', 'base_link'],
            ),
            
            # Base link to base_link_frd (ENU to NED conversion) - FROM ORIGINAL
            Node(
                package='tf2_ros',
                name='base_link_to_frd_tf_node',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'base_link', 'base_link_frd'],
            ),
        ]
    )
    
    # ============================================================================
    # STEP 3: DRONE SENSOR BRIDGE
    # ============================================================================
    
    # Drone ROS-Gazebo Bridge for Sensors
    drone_sensor_bridge = TimerAction(
        period=6.0,  # After TF setup is complete
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
                    '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                    '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                    
                    # Gimbal control
                    '/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double',
                    '/gimbal/cmd_roll@std_msgs/msg/Float64]gz.msgs.Double',
                    '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
                ],
                # Remap topics to drone namespace
                remappings=[
                    ('/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image', '/drone/gimbal_camera'),
                    ('/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info', '/drone/gimbal_camera_info'),
                    ('/gimbal/cmd_yaw', '/drone/gimbal/cmd_yaw'),
                    ('/gimbal/cmd_roll', '/drone/gimbal/cmd_roll'),
                    ('/gimbal/cmd_pitch', '/drone/gimbal/cmd_pitch'),
                    ('/imu_gimbal', '/drone/imu_gimbal'),
                    ('/scan', '/drone/scan'),
                    ('/scan/points', '/drone/scan/points'),
                    ('/world/default/model/x500_lidar_camera_1/link/base_link/sensor/air_pressure_sensor/air_pressure', '/drone/air_pressure'),
                    ('/navsat', '/drone/gps'),
                ],
            )
        ]
    )
    
    # ============================================================================
    # STEP 4: STATUS AND MONITORING
    # ============================================================================
    
    # Drone Status Check
    drone_status_check = TimerAction(
        period=8.0,  # Check status after all drone components should be loaded
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Drone SAR System Status:' && ros2 topic list | grep drone && echo 'MAVROS Topics:' && ros2 topic list | grep mavros"],
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
        
        # Step 1: Drone spawn and systems (connects to existing Gazebo)
        drone_delayed_spawn,
        
        # Step 2: Drone TF transforms
        drone_tf_transforms,
        
        # Step 3: Drone sensor bridge
        drone_sensor_bridge,
        
        # Step 4: Status monitoring
        drone_status_check,
    ])