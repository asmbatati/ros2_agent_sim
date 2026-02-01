# ROS 2 Agent Simulation Framework

A scalable ROS 2-based framework for simulating and controlling heterogeneous multi-robot systems (UAVs, Quadrupeds) using Large Language Models (LLMs).

## Features
- **Dynamic Simulation**: Spawn `n` robots (Unitree Go2, PX4 Drones) at runtime via a web-based GUI.
- **Agent Abstraction**: Unified ROS 2 Action interface for controlling diverse robots.
- **LLM Integration**: Natural language control of robot teams (e.g., "Drone 1 investigate the red box, Go2 2 follow it").
- **Scalable**: Optimized for multi-agent performance in Gazebo Harmonic.

## Installation

Ensure you are in the Docker container or a ROS 2 Jazzy environment.

```bash
cd ~/shared_volume/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Workflow

### 1. Launch the Simulation GUI
Start the simulation orchestration node. This hosts the web interface for scenario configuration.

```bash
ros2 run simulation_gui gui_node
```
*   **Access the GUI**: Open your browser at `http://localhost:3000`.
    *   *Note for VS Code Users*: If running in a container, you must forward the port:
        1.  Press `Ctrl + Shift + P`.
        2.  Type and select **"Ports: Focus on Ports View"**.
        3.  Add port `3000` (and `8000` if needed for API access).
        4.  Click the globe icon to open the address in your browser.

### 2. Configure and Launch Scenario
1.  **Drag & Drop**: Use the sidebar to drag robots (Go2, Drone) and assets (Boxes, People) onto the map.
2.  **Launch**: Click the "Launch Simulation" button.
    *   This will automatically generate a launch file, kill existing simulations, and start Gazebo Harmonic + RViz with your configuration.

### 3. Run the Evaluation Logger (Optional)
To log Real Time Factor (RTF) and system metrics:

```bash
ros2 run sim_evaluation eval_logger
```

### 4. Run the AI Agent
Start the LLM-driven agent node. This node discovers all active robots and accepts natural language commands.

```bash
# Basic run
ros2 run ros2_agent agent_node

# With specific model (if configured)
ros2 run ros2_agent agent_node --ros-args -p llm_model:=gemini-1.5-flash
```

### 5. Control the Team
Once the agent is running, you can type commands in the terminal (or connect via the future chat interface):

*   *"Discover available robots"* -> Lists tracked agents (e.g., `go2_1`, `drone_1`).
*   *"Drone 1, takeoff to 5 meters."*
*   *"Go2 1, move forward 2 meters."*
*   *"All robots, stop."*

## Architecture Modules
*   **`simulation_gui`**: React frontend + FastAPI/ROS 2 backend for orchestration.
*   **`ros2_agent`**: Core agent logic, tool definitions, and LLM integration.
*   **`sim_evaluation`**: Benchmarking tools for RTF and latency.
