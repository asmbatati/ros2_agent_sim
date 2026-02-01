#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, Body
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
import subprocess
import json
from ament_index_python.packages import get_package_share_directory

# FastAPI App
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global reference to ROS node
ros_node = None

class GuiNode(Node):
    def __init__(self):
        super().__init__('simulation_gui_node')
        self.get_logger().info('Simulation GUI Node Started on http://localhost:8000')

    def launch_sim(self, items):
        self.get_logger().info(f'Launching simulation with items: {items}')
        
        # Generator for unique IDs
        go2_count = 0
        drone_count = 0
        
        # Prepare the launch file content dynamically
        launch_content = """
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    px4_dir = '/home/user/shared_volume/PX4-Autopilot'
    world_file = f'{px4_dir}/Tools/simulation/gz/worlds/default.sdf'

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': f'{world_file} -r', 'on_exit_shutdown': 'true'}.items()
    )

    # 2. Bridge (Dynamic Layout)
    bridge_config = [
        # Clock (Essential)
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    
    # 3. Spawns
    spawn_actions = []
"""
        
        # Iterate and add robot logic
        for item in items:
            x, y = item.get('x', 0), item.get('y', 0)
            spawn_z = 0.4 # Default safe height
            
            if item['type'] == 'robot':
                if item['id'] == 'go2':
                    go2_count += 1
                    name = f"go2_{go2_count}"
                    namespace = f"/{name}"
                    
                    launch_content += f"""
    # --- Unitree Go2: {name} ---
    # Process Xacro
    go2_desc_path = os.path.join(get_package_share_directory("unitree_go2_description"), "urdf/unitree_go2_robot.xacro")
    go2_urdf = Command(['xacro ', go2_desc_path])
    
    go2_{go2_count}_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', '{name}', '-x', '{x}', '-y', '{y}', '-z', '0.4', '-Y', '0.0', '-string', go2_urdf],
        output='screen'
    )
    spawn_actions.append(go2_{go2_count}_spawn)
    
    # Add Bridge topics for {name}
    bridge_config.extend([
        '{namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '{namespace}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '{namespace}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '{namespace}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '{namespace}/rgb_image@sensor_msgs/msg/Image[gz.msgs.Image'
    ])
"""
                elif item['id'] == 'drone':
                    drone_count += 1
                    name = f"drone_{drone_count}" # PX4 usually requires specific IDs, simplifying for generic spawn
                    # Note: PX4 SITL is complex to multi-spawn without unique instance IDs/ports. 
                    # For this prototype, we will spawn the model but full flight control might be limited to one if using standard SITL.
                    # We will use the generic x500 model spawn.
                    
                    launch_content += f"""
    # --- Drone: {name} ---
    drone_{drone_count}_spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', '{name}', '-x', '{x}', '-y', '{y}', '-z', '0.5', '-Y', '0.0', 
                   '-file', f'{{px4_dir}}/Tools/simulation/gz/models/x500/model.sdf'],
        output='screen'
    )
    spawn_actions.append(drone_{drone_count}_spawn)
"""

            elif item['type'] == 'asset':
                asset_id = item['id']
                unique_name = f"{asset_id}_{item['instanceId']}"
                
                # Determine model file/uri
                model_uri = ""
                if asset_id == "person":
                    model_uri = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"
                elif asset_id == "box":
                    model_uri = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Wood Cube 10cm"
                
                if model_uri:
                    launch_content += f"""
    # --- Asset: {unique_name} ---
    spawn_actions.append(Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', '{unique_name}', '-x', '{x}', '-y', '{y}', '-z', '0', '-uri', '{model_uri}'],
        output='screen'
    ))
"""

        # Finalize launch file
        launch_content += """
    # Bridge Node
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'qos_overrides./clock.publisher.durability': 'transient_local'}],
        arguments=bridge_config,
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('ros2_agent'), 'rviz', 'agent.rviz')],
        condition=None,
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        rviz,
        *spawn_actions
    ])
"""

        # Write to temp file
        launch_file_path = "/tmp/generated_launch.py"
        with open(launch_file_path, "w") as f:
            f.write(launch_content)
            
        self.get_logger().info(f"Generated launch file at {launch_file_path}")
        
        # kill previous ros2 launch if possible (simple version)
        subprocess.run(["pkill", "-f", "ros2 launch"]) 
        
        # Execute
        subprocess.Popen(["ros2", "launch", launch_file_path])
        return "Launch invoked"

@app.post("/launch")
async def launch(config: dict = Body(...)):
    if ros_node:
        items = config.get('items', [])
        ros_node.launch_sim(items)
        return {"status": "success", "launch_id": "dynamic"}
    return {"status": "error", "message": "ROS Node not initialized"}

# Mount static files (Frontend)
try:
    pkg_share = get_package_share_directory('simulation_gui')
    web_dir = os.path.join(pkg_share, 'web')
    if os.path.exists(web_dir):
        app.mount("/", StaticFiles(directory=web_dir, html=True), name="static")
    else:
        # Fallback for local development
        dev_web_dir = os.path.abspath(os.path.join(os.getcwd(), 'src/ros2_agent_sim/simulation_gui/web/dist'))
        if os.path.exists(dev_web_dir):
            app.mount("/", StaticFiles(directory=dev_web_dir, html=True), name="static")
except Exception as e:
    print(f"Error locating web directory: {e}")

# Global reference for server
server = None

def run_uvicorn():
    global server
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info", loop="asyncio")
    server = uvicorn.Server(config)
    server.run()

def main(args=None):
    global ros_node, server
    rclpy.init(args=args)
    ros_node = GuiNode()
    
    # Start Uvicorn in a separate thread
    server_thread = threading.Thread(target=run_uvicorn, daemon=True)
    server_thread.start()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown Uvicorn cleanly
        if server:
            server.should_exit = True
            server_thread.join(timeout=2)
            
        # Clean up ROS node
        if ros_node:
            ros_node.destroy_node()
            
        # Shutdown rclpy only if context is valid
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
