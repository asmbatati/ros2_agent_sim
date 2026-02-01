#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time
import csv
import os
from datetime import datetime

class EvalLogger(Node):
    def __init__(self):
        super().__init__('eval_logger')
        
        self.start_wall_time = time.time()
        self.start_sim_time = None
        self.last_sim_time = 0.0
        self.last_wall_time = time.time()
        
        # Metrics
        self.robot_count = 0
        self.rtf = 1.0
        
        # CSV Logging
        self.log_dir = os.path.expanduser('~/shared_volume/publications/RSS paper/data')
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_file = os.path.join(self.log_dir, f'evaluation_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'WallTime', 'SimTime', 'RobotCount', 'RTF'])
            
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)
            
        # Timer for periodic logging (1Hz)
        self.create_timer(1.0, self.log_metrics)
        
        self.get_logger().info(f"Evaluation Logger Started. Logging to {self.csv_file}")

    def clock_callback(self, msg):
        current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        
        if self.start_sim_time is None:
            self.start_sim_time = current_sim_time
            
        # Update RTF calculation window
        current_wall_time = time.time()
        wall_delta = current_wall_time - self.last_wall_time
        sim_delta = current_sim_time - self.last_sim_time
        
        if wall_delta > 1.0: # Calculate RTF over 1s windows
            self.rtf = sim_delta / wall_delta
            self.last_wall_time = current_wall_time
            self.last_sim_time = current_sim_time

    def count_robots(self):
        # Heuristic: Count number of '/go2_{n}/scan' or '/drone_{n}/odometry' topics
        # This is a bit expensive so we do it in the 1Hz timer
        topics = self.get_topic_names_and_types()
        count = 0
        known_agents = set()
        
        for name, types in topics:
            if '/scan' in name and 'go2' in name:
                agent_name = name.split('/')[1]
                known_agents.add(agent_name)
            if '/odom' in name and 'drone' in name:
                agent_name = name.split('/')[1]
                known_agents.add(agent_name)
                
        self.robot_count = len(known_agents)

    def log_metrics(self):
        self.count_robots()
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        wall_duration = time.time() - self.start_wall_time
        sim_duration = self.last_sim_time - (self.start_sim_time if self.start_sim_time else 0)
        
        self.get_logger().info(f"Agents: {self.robot_count} | RTF: {self.rtf:.2f}")
        
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, wall_duration, sim_duration, self.robot_count, self.rtf])

def main(args=None):
    rclpy.init(args=args)
    node = EvalLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
