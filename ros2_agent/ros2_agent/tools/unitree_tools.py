#!/usr/bin/env python3
"""
Unitree Go2 tools for the ROSA Agent.
This module contains tools for quadruped robot control with improved straight-line movement.
"""

import time
import threading
from langchain.agents import tool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

class UnitreeTools:   
    """Collection of tools for Unitree Go2 quadruped robot control."""
    
    def __init__(self, node, namespace='/go2'):
        self.node = node
        self.namespace = namespace.rstrip('/')
        self._setup_go2_subscribers()
        
    def _setup_go2_subscribers(self):
        """Setup subscribers for Go2 robot data."""
        # Initialize Go2 pose tracking
        setattr(self.node, f'current_go2_pose_{self.namespace}', Odometry())
        setattr(self.node, f'go2_nav_active_{self.namespace}', False)
        
        # Create odometry subscriber
        sub_attr = f'go2_odom_sub_{self.namespace}'
        if not hasattr(self.node, sub_attr):
            def odom_callback(msg):
                setattr(self.node, f'current_go2_pose_{self.namespace}', msg)
                
            setattr(self.node, sub_attr, self.node.create_subscription(
                Odometry,
                f'{self.namespace}/odom',
                odom_callback,
                10
            ))
            self.node.get_logger().info(f"Created Go2 odometry subscriber for {self.namespace}")
        
        # Add helper method to node
        def get_go2_yaw():
            """Extract yaw angle from Go2 quaternion orientation."""
            orientation = self.node.current_go2_pose.pose.pose.orientation
            
            # Convert quaternion to yaw angle
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w
            
            # Calculate yaw from quaternion
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            return yaw
            
        self.node.get_go2_yaw = get_go2_yaw
        
    def _go2_odom_callback(self, msg):
        """Callback for Go2 odometry updates (maintained for compatibility)."""
        setattr(self.node, f'current_go2_pose_{self.namespace}', msg)
        
    def create_tools(self):
        """Create and return all Unitree Go2 tools."""
        
        # Reference to the node for use in the closure
        node = self.node
        
        @tool
        def go2_move_forward(distance: float, speed: float = 0.3) -> str:
            """
            Move the Go2 robot forward by a specific distance in a straight line.
            Uses closed-loop control with odometry feedback for accuracy.
            
            Use this for commands like:
            - "go2 move 3 steps forward" (distance = 3.0)
            - "go2 move forward 2 meters" (distance = 2.0)
            - "go2 walk forward" (distance = 1.0 default)
            
            Args:
                distance: Distance to move forward in meters (1 step = 1 meter)
                speed: Movement speed in m/s (default: 0.3 for better control)
            
            Returns:
                str: Status message about the movement command
            """
            # Input validation
            if distance <= 0:
                return "Error: Distance must be positive"
            if distance > 10:
                return "Error: Distance too large (max 10 meters for safety)"
            if speed <= 0 or speed > 1.0:
                return "Error: Speed must be between 0.1 and 1.0 m/s"
            
            # Get starting position and orientation
            current_pose = getattr(node, f'current_go2_pose_{self.namespace}')
            start_x = current_pose.pose.pose.position.x
            start_y = current_pose.pose.pose.position.y
            start_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
            
            # Calculate target position based on starting heading
            target_x = start_x + distance * math.cos(start_yaw)
            target_y = start_y + distance * math.sin(start_yaw)
            
            node.get_logger().info(f"Go2 straight-line movement: {distance:.1f}m from ({start_x:.2f}, {start_y:.2f}) to ({target_x:.2f}, {target_y:.2f})")
            node.get_logger().info(f"Initial heading: {math.degrees(start_yaw):.1f}°")
            
            # Create cmd_vel publisher if it doesn't exist
            pub_attr = f'go2_cmd_vel_pub_{self.namespace}'
            if not hasattr(node, pub_attr):
                setattr(node, pub_attr, node.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10))
                node.get_logger().info(f"Created cmd_vel publisher for {self.namespace}")
            
            cmd_vel_pub = getattr(node, pub_attr)
            
            # Straight-line movement with feedback control
            def straight_line_movement():
                rate = node.create_rate(20)  # 20 Hz control loop
                tolerance = 0.1  # 10cm tolerance for completion
                last_log_time = time.time()
                
                while node.running:
                    # Get current position
                    curr_pose = getattr(node, f'current_go2_pose_{self.namespace}')
                    curr_x = curr_pose.pose.pose.position.x
                    curr_y = curr_pose.pose.pose.position.y
                    curr_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
                    
                    # Calculate distance traveled and remaining
                    dx_traveled = curr_x - start_x
                    dy_traveled = curr_y - start_y
                    distance_traveled = math.sqrt(dx_traveled*dx_traveled + dy_traveled*dy_traveled)
                    remaining_distance = distance - distance_traveled
                    
                    # Check if we've reached the target distance
                    if remaining_distance <= tolerance or distance_traveled >= distance:
                        # Stop the robot
                        stop_cmd = Twist()
                        for _ in range(5):
                            cmd_vel_pub.publish(stop_cmd)
                            time.sleep(0.05)
                        
                        final_x = getattr(node, f'current_go2_pose_{self.namespace}').pose.pose.position.x
                        final_y = getattr(node, f'current_go2_pose_{self.namespace}').pose.pose.position.y
                        final_distance = math.sqrt((final_x - start_x)**2 + (final_y - start_y)**2)
                        
                        node.get_logger().info(f"Go2 straight-line movement completed!")
                        node.get_logger().info(f"Distance traveled: {final_distance:.2f}m (target: {distance:.2f}m)")
                        break
                    
                    # Calculate heading error (deviation from initial heading)
                    heading_error = start_yaw - curr_yaw
                    
                    # Normalize heading error to [-pi, pi]
                    while heading_error > math.pi:
                        heading_error -= 2 * math.pi
                    while heading_error < -math.pi:
                        heading_error += 2 * math.pi
                    
                    # Create movement command with heading correction
                    cmd = Twist()
                    
                    # Forward speed with deceleration as approaching target
                    if remaining_distance > 1.0:
                        cmd.linear.x = speed
                    else:
                        # Slow down when close to target
                        cmd.linear.x = max(0.1, speed * (remaining_distance / 1.0))
                    
                    # Gentle heading correction to maintain straight line
                    # Only correct if heading error is significant
                    if abs(heading_error) > 0.05:  # ~3 degrees threshold
                        cmd.angular.z = heading_error * 0.2  # Gentle proportional control
                        if time.time() - last_log_time > 2.0:  # Log corrections every 2 seconds
                            node.get_logger().info(f"Heading correction: error={math.degrees(heading_error):.1f}°, correction={cmd.angular.z:.3f}")
                            last_log_time = time.time()
                    else:
                        cmd.angular.z = 0.0
                    
                    # Limit angular velocity for stability
                    cmd.angular.z = max(-0.3, min(0.3, cmd.angular.z))
                    
                    # Publish command
                    cmd_vel_pub.publish(cmd)
                    
                    # Log progress every 1 second
                    if time.time() - last_log_time > 1.0:
                        node.get_logger().info(f"Progress: {distance_traveled:.2f}m / {distance:.2f}m")
                        last_log_time = time.time()
                    
                    rate.sleep()
            
            # Start movement thread
            movement_thread = threading.Thread(target=straight_line_movement)
            movement_thread.daemon = True
            movement_thread.start()
            
            return f"✅ Go2 straight-line movement started: {distance:.1f}m\n• Speed: {speed:.1f} m/s\n• Target heading: {math.degrees(start_yaw):.1f}°\n• Feedback control: ACTIVE"
            
        @tool
        def go2_move_to_position(x: float, y: float, speed: float = 0.5) -> str:
            """
            Move the Go2 robot to a specific position (x, y) in local coordinates.
            
            Args:
                x: Target X position in meters
                y: Target Y position in meters  
                speed: Movement speed in m/s (default: 0.5, max: 1.0)
            
            Returns:
                str: Status message about the movement command
            """
            # Input validation
            if speed <= 0 or speed > 1.0:
                return "Error: Speed must be between 0.1 and 1.0 m/s"
            
            # Get current position
            current_pose = getattr(node, f'current_go2_pose_{self.namespace}')
            current_x = current_pose.pose.pose.position.x
            current_y = current_pose.pose.pose.position.y
            current_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
            
            node.get_logger().info(f"Go2 moving from ({current_x:.2f}, {current_y:.2f}) to ({x:.2f}, {y:.2f})")
            
            # Calculate distance and direction
            dx = x - current_x
            dy = y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            target_yaw = math.atan2(dy, dx)
            
            if distance < 0.1:
                return f"Go2 is already at target position ({x:.2f}, {y:.2f})"
            
            # Create cmd_vel publisher if it doesn't exist
            pub_attr = f'go2_cmd_vel_pub_{self.namespace}'
            if not hasattr(node, pub_attr):
                setattr(node, pub_attr, node.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10))
                node.get_logger().info(f"Created cmd_vel publisher for {self.namespace}")
            
            cmd_vel_pub = getattr(node, pub_attr)
            
            # Store target and start navigation
            setattr(node, f'go2_target_x_{self.namespace}', x)
            setattr(node, f'go2_target_y_{self.namespace}', y)
            setattr(node, f'go2_nav_active_{self.namespace}', True)
            
            # Start navigation thread with improved control
            def navigation_thread():
                rate = node.create_rate(10)  # 10 Hz for stability
                tolerance = 0.15  # 15cm tolerance
                
                # Control variables
                previous_yaw_error = 0.0
                
                while getattr(node, f'go2_nav_active_{self.namespace}', False) and node.running:
                    # Get current position
                    curr_pose = getattr(node, f'current_go2_pose_{self.namespace}')
                    curr_x = curr_pose.pose.pose.position.x
                    curr_y = curr_pose.pose.pose.position.y
                    curr_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
                    
                    # Calculate remaining distance
                    target_x = getattr(node, f'go2_target_x_{self.namespace}')
                    target_y = getattr(node, f'go2_target_y_{self.namespace}')
                    dx = target_x - curr_x
                    dy = target_y - curr_y
                    remaining_distance = math.sqrt(dx*dx + dy*dy)
                    
                    # Check if reached target
                    if remaining_distance < tolerance:
                        # Stop robot with multiple commands
                        stop_cmd = Twist()
                        for _ in range(5):
                            cmd_vel_pub.publish(stop_cmd)
                            time.sleep(0.05)
                        setattr(node, f'go2_nav_active_{self.namespace}', False)
                        node.get_logger().info(f"Go2 reached target! Final distance: {remaining_distance:.2f}m")
                        break
                    
                    # Calculate desired heading
                    desired_yaw = math.atan2(dy, dx)
                    yaw_error = desired_yaw - curr_yaw
                    
                    # Normalize yaw error to [-pi, pi]
                    while yaw_error > math.pi:
                        yaw_error -= 2 * math.pi
                    while yaw_error < -math.pi:
                        yaw_error += 2 * math.pi
                    
                    # Create movement command with improved control
                    cmd = Twist()
                    
                    # Decide: turn first or move with course correction
                    if abs(yaw_error) > 0.3:  # ~17 degrees - significant heading error
                        # Turn in place first
                        cmd.angular.z = 0.2 if yaw_error > 0 else -0.2  # Reduced from 0.3
                        cmd.linear.x = 0.0
                        node.get_logger().info(f"Turning to target: yaw_error={math.degrees(yaw_error):.1f}°")
                        
                    else:
                        # Move forward with gentle course correction
                        
                        # Forward speed control with distance-based scaling
                        if remaining_distance > 2.0:
                            cmd.linear.x = speed
                        elif remaining_distance > 0.5:
                            cmd.linear.x = speed * 0.7  # Slow down when getting close
                        else:
                            cmd.linear.x = speed * 0.4  # Very slow when very close
                        
                        # Gentle course correction using proportional control
                        kp_yaw = 0.3  # Proportional gain (reduced from 0.5)
                        cmd.angular.z = yaw_error * kp_yaw
                        
                        # Limit angular velocity to prevent oscillation
                        max_angular = 0.2  # Reduced from 0.3
                        cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))
                        
                        # Reduce angular correction when moving fast
                        if cmd.linear.x > 0.3:
                            cmd.angular.z *= 0.5  # Reduce turning when moving fast
                    
                    # Publish command
                    cmd_vel_pub.publish(cmd)
                    
                    # Update previous values for next iteration
                    previous_yaw_error = yaw_error
                    
                    rate.sleep()
            
            # Start navigation thread
            nav_thread = threading.Thread(target=navigation_thread)
            nav_thread.daemon = True
            nav_thread.start()
            
            return f"✅ Go2 navigating to position ({x:.2f}, {y:.2f})\n• Distance: {distance:.2f}m\n• Speed: {speed:.1f} m/s\n• Navigation active: ✓"
            
        @tool
        def go2_stop() -> str:
            """
            Stop the Go2 robot immediately and cancel any active navigation.
            
            Returns:
                str: Status message about the stop command
            """
            node.get_logger().info("Emergency stop for Go2 robot")
            
            try:
                # Create publisher if it doesn't exist
                pub_attr = f'go2_cmd_vel_pub_{self.namespace}'
                if not hasattr(node, pub_attr):
                    setattr(node, pub_attr, node.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10))
                cmd_vel_pub = getattr(node, pub_attr)
                
                # Stop navigation
                setattr(node, f'go2_nav_active_{self.namespace}', False)
                
                # Send stop command multiple times for safety
                stop_cmd = Twist()  # All zeros
                for _ in range(5):
                    cmd_vel_pub.publish(stop_cmd)
                    time.sleep(0.01)
                
                return "✅ Go2 robot stopped!\n• All movement commands canceled\n• Navigation disabled\n• Robot is now stationary"
                
            except Exception as e:
                return f"Error stopping Go2: {str(e)}"
            
        @tool
        def get_go2_position() -> str:
            """
            Get the current position and orientation of the Go2 robot.
            
            Returns:
                str: Current robot position (x, y, orientation) information
            """
            node.get_logger().info("Getting Go2 robot position...")
            
            try:
                # Get current pose from odometry
                current_pose = getattr(node, f'current_go2_pose_{self.namespace}')
                
                # Extract position
                x = current_pose.pose.pose.position.x
                y = current_pose.pose.pose.position.y
                z = current_pose.pose.pose.position.z
                
                # Extract orientation and convert to yaw
                yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
                yaw_degrees = math.degrees(yaw)
                
                # Format response
                pose_info = (
                    f"🐕 Go2 Robot Current Position:\n\n"
                    f"📍 Position:\n"
                    f"  • X: {x:.2f} m\n"
                    f"  • Y: {y:.2f} m\n"
                    f"  • Z: {z:.2f} m\n\n"
                    f"🧭 Orientation:\n"
                    f"  • Yaw: {yaw_degrees:.1f}°\n"
                    f"  • Heading: {yaw:.3f} rad\n\n"
                    f"📊 Status: Position data retrieved successfully"
                )
                
                return pose_info
                
            except Exception as e:
                return f"Error getting Go2 position: {str(e)}"
            
        @tool
        def go2_camera_feed(action: str = "start") -> str:
            """
            Control Go2 camera feed display in a pop-up window.
            
            Args:
                action: "start" to open camera feed, "stop" to close camera feed
            
            Returns:
                str: Status of camera feed operation
            """
            
            if action.lower() == "start":
                node.get_logger().info("Starting Go2 camera feed...")
                
                # Initialize camera components if needed
                bridge_attr = f'go2_camera_bridge_{self.namespace}'
                active_attr = f'go2_camera_active_{self.namespace}'
                latest_frame_attr = f'go2_latest_frame_{self.namespace}'
                
                if not hasattr(node, bridge_attr):
                    setattr(node, bridge_attr, CvBridge())
                    setattr(node, active_attr, False)
                    setattr(node, latest_frame_attr, None)
                    
                # Create camera subscriber if it doesn't exist
                sub_attr = f'go2_camera_sub_{self.namespace}'
                if not hasattr(node, sub_attr):
                    def go2_camera_callback(msg):
                        try:
                            # Convert ROS image to OpenCV
                            bridge = getattr(node, bridge_attr)
                            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                            # Resize for display
                            resized_image = cv2.resize(cv_image, (640, 480))
                            setattr(node, latest_frame_attr, resized_image)
                        except Exception as e:
                            node.get_logger().error(f"Go2 camera callback error: {str(e)}")
                    
                    setattr(node, sub_attr, node.create_subscription(
                        Image, f'{self.namespace}/rgb_image', go2_camera_callback, 10))
                    node.get_logger().info(f"Created Go2 camera subscriber for {self.namespace}")
                
                # Start camera display thread
                if not getattr(node, active_attr):
                    setattr(node, active_attr, True)
                    
                    def go2_camera_display_thread():
                        window_name = f"Go2 Camera Feed ({self.namespace})"
                        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(window_name, 640, 480)
                        
                        while getattr(node, active_attr) and node.running:
                            frame = getattr(node, latest_frame_attr)
                            if frame is not None:
                                # Add timestamp overlay
                                frame_with_overlay = frame.copy()
                                timestamp = time.strftime("%H:%M:%S")
                                cv2.putText(frame_with_overlay, f"Go2 CAM {timestamp}", 
                                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                
                                cv2.imshow(window_name, frame_with_overlay)
                            
                            if cv2.waitKey(33) & 0xFF == ord('q'):
                                break
                        
                        cv2.destroyWindow(window_name)
                    
                    thread = threading.Thread(target=go2_camera_display_thread)
                    thread.daemon = True
                    thread.start()
                    setattr(node, f'go2_camera_thread_{self.namespace}', thread)
                    
                    return (
                        f"✅ Go2 Camera feed started!\n\n"
                        f"📺 Window: 640x480 resolution\n"
                        f"🎥 Live feed from Go2 camera\n"
                        f"⏱️ Timestamp overlay enabled\n\n"
                        f"Press 'q' in camera window to close or use 'stop' command."
                    )
                else:
                    return "Go2 camera feed is already active."
                    
            elif action.lower() == "stop":
                node.get_logger().info("Stopping Go2 camera feed...")
                active_attr = f'go2_camera_active_{self.namespace}'
                
                if hasattr(node, active_attr) and getattr(node, active_attr):
                    setattr(node, active_attr, False)
                    
                    # Wait for thread to finish
                    thread_attr = f'go2_camera_thread_{self.namespace}'
                    if hasattr(node, thread_attr):
                        thread = getattr(node, thread_attr)
                        if thread.is_alive():
                            thread.join(timeout=2.0)
                    
                    # Destroy OpenCV windows
                    cv2.destroyWindow(f"Go2 Camera Feed ({self.namespace})")
                    
                    return (
                        f"✅ Go2 Camera feed stopped!\n\n"
                        f"📺 Camera window closed\n"
                        f"🎥 Video stream disconnected\n\n"
                        f"Ready for next camera operation."
                    )
                else:
                    return "No active Go2 camera feed to stop."
            
            else:
                return f"Error: Invalid action '{action}'. Use 'start' or 'stop'."
        
        @tool
        def go2_test_straight_line(distance: float = 2.0, speed: float = 0.2) -> str:
            """
            Test Go2 straight-line movement with detailed logging for diagnostics.
            
            Args:
                distance: Test distance in meters (default: 2.0)
                speed: Test speed in m/s (default: 0.2 - slow for observation)
            
            Returns:
                str: Detailed diagnostic information about the movement
            """
            # Record starting position and heading
            current_pose = getattr(node, f'current_go2_pose_{self.namespace}')
            start_x = current_pose.pose.pose.position.x
            start_y = current_pose.pose.pose.position.y
            start_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
            
            node.get_logger().info(f"=== GO2 STRAIGHT-LINE DIAGNOSTIC TEST ===")
            node.get_logger().info(f"Start position: ({start_x:.3f}, {start_y:.3f})")
            node.get_logger().info(f"Start heading: {math.degrees(start_yaw):.1f}°")
            node.get_logger().info(f"Target distance: {distance}m at {speed}m/s")
            
            # Create cmd_vel publisher if needed
            pub_attr = f'go2_cmd_vel_pub_{self.namespace}'
            if not hasattr(node, pub_attr):
                setattr(node, pub_attr, node.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10))
            cmd_vel_pub = getattr(node, pub_attr)
            
            def diagnostic_movement():
                rate = node.create_rate(10)  # 10 Hz for detailed logging
                start_time = time.time()
                last_log_time = start_time
                
                while time.time() - start_time < (distance / speed + 2.0):  # Allow extra time
                    current_time = time.time() - start_time
                    
                    # Record current state
                    curr_pose = getattr(node, f'current_go2_pose_{self.namespace}')
                    curr_x = curr_pose.pose.pose.position.x
                    curr_y = curr_pose.pose.pose.position.y
                    curr_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
                    
                    # Calculate distance traveled
                    dx_traveled = curr_x - start_x
                    dy_traveled = curr_y - start_y
                    distance_traveled = math.sqrt(dx_traveled*dx_traveled + dy_traveled*dy_traveled)
                    
                    # Check if target reached
                    if distance_traveled >= distance:
                        # Stop and record final position
                        stop_cmd = Twist()
                        for _ in range(3):
                            cmd_vel_pub.publish(stop_cmd)
                            time.sleep(0.1)
                        
                        # Calculate deviation from straight line
                        expected_x = start_x + distance * math.cos(start_yaw)
                        expected_y = start_y + distance * math.sin(start_yaw)
                        lateral_error = abs((curr_y - start_y) * math.cos(start_yaw) - (curr_x - start_x) * math.sin(start_yaw))
                        
                        node.get_logger().info(f"=== DIAGNOSTIC RESULTS ===")
                        node.get_logger().info(f"Final position: ({curr_x:.3f}, {curr_y:.3f})")
                        node.get_logger().info(f"Expected position: ({expected_x:.3f}, {expected_y:.3f})")
                        node.get_logger().info(f"Distance traveled: {distance_traveled:.3f}m (target: {distance:.3f}m)")
                        node.get_logger().info(f"Lateral deviation: {lateral_error:.3f}m")
                        node.get_logger().info(f"Final heading: {math.degrees(curr_yaw):.1f}° (start: {math.degrees(start_yaw):.1f}°)")
                        break
                    
                    # Create movement command
                    cmd = Twist()
                    cmd.linear.x = speed
                    
                    # Heading correction to maintain straight line
                    heading_error = start_yaw - curr_yaw
                    while heading_error > math.pi:
                        heading_error -= 2 * math.pi
                    while heading_error < -math.pi:
                        heading_error += 2 * math.pi
                    
                    # Very gentle correction
                    cmd.angular.z = heading_error * 0.1
                    
                    # Log detailed info every second
                    if time.time() - last_log_time >= 1.0:
                        node.get_logger().info(f"t={current_time:.1f}s: pos=({curr_x:.3f},{curr_y:.3f}), "
                                             f"dist={distance_traveled:.3f}m, "
                                             f"heading={math.degrees(curr_yaw):.1f}°, "
                                             f"h_err={math.degrees(heading_error):.1f}°")
                        last_log_time = time.time()
                    
                    cmd_vel_pub.publish(cmd)
                    rate.sleep()
            
            # Start diagnostic movement
            movement_thread = threading.Thread(target=diagnostic_movement)
            movement_thread.daemon = True
            movement_thread.start()
            
            return f"✅ Go2 diagnostic test started!\n• Distance: {distance}m\n• Speed: {speed}m/s\n• Detailed logging: ACTIVE\n• Check logs for real-time analysis"

        @tool
        def go2_move_circle(radius: float = 1.0, angular_speed: float = 0.3, duration: float = 10.0) -> str:
            """
            Move the Go2 robot in a circular motion using velocity commands.
            
            Args:
                radius: Radius of the circular path in meters (default: 1.0m)
                angular_speed: Angular speed in rad/s (default: 0.3 rad/s, range: 0.1–1.0)
                duration: Duration of motion in seconds (default: 10.0s)
            
            Returns:
                str: Status message about the circular motion command
            """
            if radius <= 0:
                return "Error: Radius must be positive"
            if angular_speed <= 0 or angular_speed > 1.0:
                return "Error: Angular speed must be between 0.1 and 1.0 rad/s"
            if duration <= 0 or duration > 60:
                return "Error: Duration must be between 1 and 60 seconds"

            # Create publisher if it doesn't exist
            pub_attr = f'go2_cmd_vel_pub_{self.namespace}'
            if not hasattr(node, pub_attr):
                setattr(node, pub_attr, node.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10))
                node.get_logger().info(f"Created Go2 cmd_vel publisher for {self.namespace}")
            cmd_vel_pub = getattr(node, pub_attr)

            linear_speed = radius * angular_speed

            def circular_motion():
                rate = node.create_rate(20)  # 20 Hz
                start_time = time.time()

                cmd = Twist()
                cmd.linear.x = linear_speed
                cmd.angular.z = angular_speed

                while node.running and (time.time() - start_time < duration):
                    cmd_vel_pub.publish(cmd)
                    rate.sleep()

                # Stop after motion
                stop_cmd = Twist()
                for _ in range(5):
                    cmd_vel_pub.publish(stop_cmd)
                    time.sleep(0.05)

                node.get_logger().info(f"Go2 circular motion complete: radius={radius:.2f}m, duration={duration:.1f}s")

            thread = threading.Thread(target=circular_motion)
            thread.daemon = True
            thread.start()

            return (
                f"✅ Go2 circular motion started!\n"
                f"• Radius: {radius:.2f} m\n"
                f"• Angular speed: {angular_speed:.2f} rad/s\n"
                f"• Linear speed: {linear_speed:.2f} m/s\n"
                f"• Duration: {duration:.1f} seconds"
            )
            
        @tool 
        def go2_calibrate_straight() -> str:
            """
            Quick calibration to test if Go2 moves straight without correction.
            Sends pure forward command to identify mechanical bias.
            """
            node.get_logger().info("=== GO2 CALIBRATION TEST ===")
            node.get_logger().info("Testing pure forward movement (no correction)")
            
            # Record starting state
            current_pose = getattr(node, f'current_go2_pose_{self.namespace}')
            start_x = current_pose.pose.pose.position.x
            start_y = current_pose.pose.pose.position.y
            start_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
            
            pub_attr = f'go2_cmd_vel_pub_{self.namespace}'
            if not hasattr(node, pub_attr):
                setattr(node, pub_attr, node.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10))
            cmd_vel_pub = getattr(node, pub_attr)
            
            def calibration_test():
                # Move forward for 3 seconds with no angular correction
                cmd = Twist()
                cmd.linear.x = 0.2  # Slow speed
                cmd.angular.z = 0.0  # NO correction
                
                for i in range(30):  # 3 seconds at 10 Hz
                    cmd_vel_pub.publish(cmd)
                    time.sleep(0.1)
                
                # Stop
                stop_cmd = Twist()
                for _ in range(3):
                    cmd_vel_pub.publish(stop_cmd)
                    time.sleep(0.1)
                
                # Analyze results
                final_pose = getattr(node, f'current_go2_pose_{self.namespace}')
                final_x = final_pose.pose.pose.position.x
                final_y = final_pose.pose.pose.position.y
                final_yaw = getattr(node, f'get_go2_yaw_{self.namespace}')()
                
                # Calculate drift
                distance_traveled = math.sqrt((final_x - start_x)**2 + (final_y - start_y)**2)
                lateral_drift = abs((final_y - start_y) * math.cos(start_yaw) - (final_x - start_x) * math.sin(start_yaw))
                heading_drift = final_yaw - start_yaw
                
                node.get_logger().info(f"CALIBRATION RESULTS:")
                node.get_logger().info(f"Distance: {distance_traveled:.3f}m")
                node.get_logger().info(f"Lateral drift: {lateral_drift:.3f}m")
                node.get_logger().info(f"Heading drift: {math.degrees(heading_drift):.1f}°")
                
                if lateral_drift > 0.1:
                    node.get_logger().warn(f"Robot has significant lateral drift! Consider mechanical adjustment.")
                if abs(heading_drift) > 0.1:
                    node.get_logger().warn(f"Robot has significant heading drift! Check wheel/motor balance.")
            
            calibration_thread = threading.Thread(target=calibration_test)
            calibration_thread.daemon = True
            calibration_thread.start()
            
            return "✅ Go2 calibration test started!\n• Duration: 3 seconds\n• Speed: 0.2 m/s\n• No angular correction\n• Check logs for drift analysis"
            
        # Return the tools
        return [
            go2_move_forward,
            go2_move_to_position,
            go2_stop,
            get_go2_position,
            go2_camera_feed,
            go2_test_straight_line,
            go2_move_circle,
            go2_calibrate_straight
        ]