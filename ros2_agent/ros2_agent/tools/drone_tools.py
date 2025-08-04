#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone control using MAVROS.
"""

import time
import threading
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode # type: ignore
from mavros_msgs.msg import VfrHud, State # type: ignore
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ..prompts.system_prompts import SARAnalysisPrompts
from ..llm.specialist_models import call_vision_specialist
import cv2

class DroneTools:   
    """Collection of tools for drone control via MAVROS."""
    
    def __init__(self, node):
        self.node = node
        
    def create_tools(self):
        """Create and return all robot tools."""
        
        # Reference to the node for use in the closure
        node = self.node
        
        @tool
        def takeoff(altitude: float = 5.0) -> str:
            """
            Command the drone to take off to the specified altitude using MAVROS.
            
            Args:
                altitude: Target altitude in meters (default: 5.0)
            
            Returns:
                str: Status message about the takeoff command
            """
            # 1. Input validationa
            if altitude <= 0:
                return "Error: Altitude must be positive"
            if altitude > 20:
                return "Error: Altitude exceeds safe limit (20m max)"
            
            # Get current position from the drone
            current_x = node.current_pose.pose.position.x
            current_y = node.current_pose.pose.position.y
            current_z = node.current_pose.pose.position.z
            
            node.get_logger().info(f"Current position: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})")
            node.get_logger().info(f"Taking off to: ({current_x:.2f}, {current_y:.2f}, {altitude:.2f})")
            
            node.get_logger().info(f"Initiating takeoff to altitude {altitude}m...")
            
            # 2. Create service clients with better error handling
            try:
                if not hasattr(node, 'arming_client'):
                    node.arming_client = node.create_client(CommandBool, '/drone/mavros/cmd/arming')
                    node.get_logger().info("Created arming service client")
                    
                if not hasattr(node, 'mode_client'):
                    node.mode_client = node.create_client(SetMode, '/drone/mavros/set_mode')
                    node.get_logger().info("Created mode service client")
            except Exception as e:
                return f"Error creating service clients: {str(e)}"
            
            # 3. Wait for services to be available
            timeout = 5.0
            if not node.arming_client.wait_for_service(timeout_sec=timeout):
                return "Error: Arming service not available"
            if not node.mode_client.wait_for_service(timeout_sec=timeout):
                return "Error: Mode service not available"
            
            # 4. Create setpoint publisher
            if not hasattr(node, 'setpoint_pub'):
                node.setpoint_pub = node.create_publisher(
                    PoseStamped,
                    '/drone/mavros/setpoint_position/local',
                    10
                )
                node.get_logger().info("Created setpoint publisher")
            
            # 5. Create and send initial setpoints
            setpoint = PoseStamped()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = current_x
            setpoint.pose.position.y = current_y
            setpoint.pose.position.z = altitude
            setpoint.pose.orientation.w = 1.0
            
            node.get_logger().info("Sending initial setpoints...")
            for i in range(10):
                setpoint.header.stamp = node.get_clock().now().to_msg()
                node.setpoint_pub.publish(setpoint)
                time.sleep(0.1)
            
            # 6. Arm the drone with proper waiting
            node.get_logger().info("Requesting drone arming...")
            try:
                arm_request = CommandBool.Request()
                arm_request.value = True
                future = node.arming_client.call_async(arm_request)
                
                # Wait for arming response
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Error: Arming request timed out"
                    
                arm_response = future.result()
                if not arm_response.success:
                    return "Error: Failed to arm drone"
                    
                node.get_logger().info("Drone armed successfully")
            except Exception as e:
                return f"Error during arming: {str(e)}"
            
            # 7. Set OFFBOARD mode with proper waiting
            node.get_logger().info("Setting OFFBOARD mode...")
            try:
                mode_request = SetMode.Request()
                mode_request.custom_mode = "OFFBOARD"
                future = node.mode_client.call_async(mode_request)
                
                # Wait for mode response
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Error: Mode setting timed out"
                    
                mode_response = future.result()
                if not mode_response.mode_sent:
                    return "Error: Failed to set OFFBOARD mode"
                    
                node.get_logger().info("OFFBOARD mode set successfully")
            except Exception as e:
                return f"Error setting mode: {str(e)}"
            
            # 8. Store setpoint and enable publishing
            node.target_setpoint = setpoint
            node.publish_setpoints = True
            
            # 9. Start setpoint publisher thread (improved)
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                def setpoint_publisher():
                    rate = node.create_rate(10)
                    while node.running:
                        if getattr(node, 'publish_setpoints', False) and hasattr(node, 'target_setpoint'):
                            node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                            node.setpoint_pub.publish(node.target_setpoint)
                        rate.sleep()
                
                node.setpoint_thread = threading.Thread(target=setpoint_publisher)
                node.setpoint_thread.daemon = True
                node.setpoint_thread.start()
                node.get_logger().info("Started setpoint publisher thread")
            
            return f"✅ Takeoff successful! Target altitude: {altitude}m\n• Armed: ✓\n• Mode: OFFBOARD\n• Publishing setpoints: ✓"
            
        @tool
        def land() -> str:
            """
            Land the drone at its current location.
    
            Use this tool when the user wants to:
            - Land the drone
            - Bring the drone down
            - Go to the ground
            - Descend to ground level
            - Stop flying and land
            - Return to ground
            - Touch down
            - Complete landing sequence
            
            This function disables position control allowing the drone to descend naturally
            to the ground. It monitors the landing process until the drone reaches ground
            level (altitude < 0.2m).
            
            The tool automatically handles:
            - Disabling setpoint publishing 
            - Monitoring descent progress
            - Confirming successful touchdown
            - Providing detailed landing status
            
            Returns:
                str: Detailed status of the landing process including starting altitude,
                    final altitude, descent amount, and landing confirmation.
            """
            node.get_logger().info("Initiating complete landing sequence...")
            
            # 1. Get current position for reference
            current_x = node.current_pose.pose.position.x
            current_y = node.current_pose.pose.position.y
            start_altitude = node.current_pose.pose.position.z
            
            node.get_logger().info(f"Starting landing from position: ({current_x:.2f}, {current_y:.2f}, {start_altitude:.2f})")
            
            # 2. Check if we're already on the ground
            if start_altitude < 0.2:
                node.get_logger().info("Drone appears to be already on the ground")
                return f"Drone is already on ground (altitude: {start_altitude:.2f}m)."
            
            # 3. DISABLE setpoint publishing to allow natural descent/landing
            if hasattr(node, 'publish_setpoints'):
                node.publish_setpoints = False
                node.get_logger().info("✅ Disabled position control - drone will now descend to ground")
            else:
                node.get_logger().warn("No active position control found")
                return "Warning: No position control was active"
            
            # 4. Brief pause to ensure the change takes effect
            time.sleep(0.5)
            
            # 5. Monitor descent until COMPLETE landing
            timeout = 30.0  # Reduced from 45.0 seconds
            start_time = time.time()
            last_report_time = start_time
            descent_detected = False
            
            node.get_logger().info(f"Monitoring landing progress (timeout: {timeout}s)...")
            
            while time.time() - start_time < timeout:
                current_altitude = node.current_pose.pose.position.z
                elapsed_time = time.time() - start_time
                
                # Check if we've actually landed (ground level)
                if current_altitude < 0.2:
                    landing_time = elapsed_time
                    node.get_logger().info(f"🎯 LANDING COMPLETE! Final altitude: {current_altitude:.2f}m (took {landing_time:.1f}s)")
                    break
                
                # Detect if descent has started
                if not descent_detected and start_altitude - current_altitude > 0.3:
                    descent_detected = True
                    node.get_logger().info(f"✅ Descent confirmed - continuing to ground level...")
                
                # Report progress every 2 seconds (reduced from 3)
                if time.time() - last_report_time > 2.0:
                    descent_amount = start_altitude - current_altitude
                    node.get_logger().info(f"Landing progress: {current_altitude:.2f}m (descended {descent_amount:.2f}m)")
                    last_report_time = time.time()
                
                # Check more frequently (reduced from 1.0 to 0.5 seconds)
                time.sleep(0.5)
            else:
                # Timeout occurred - check final state
                current_altitude = node.current_pose.pose.position.z
                if current_altitude < 0.2:
                    node.get_logger().info("Landing completed during timeout check")
                else:
                    node.get_logger().warn(f"Landing timeout - drone still at {current_altitude:.2f}m")
            
            # 6. Get final position
            final_altitude = node.current_pose.pose.position.z
            total_descent = start_altitude - final_altitude
            
            # 7. Return comprehensive status report
            if final_altitude < 0.2:
                status = (
                    f"🎯 LANDING SUCCESSFUL!\n\n"
                    f"• Starting altitude: {start_altitude:.2f}m\n"
                    f"• Final altitude: {final_altitude:.2f}m\n"
                    f"• Total descent: {total_descent:.2f}m\n"
                    f"• Landing position: ({current_x:.2f}, {current_y:.2f})\n"
                    f"• Position control: DISABLED\n\n"
                    f"✅ Drone has landed successfully."
                )
            elif final_altitude < 1.0:
                status = (
                    f"⚠️ PARTIAL LANDING\n\n"
                    f"• Starting altitude: {start_altitude:.2f}m\n"
                    f"• Current altitude: {final_altitude:.2f}m\n"
                    f"• Descent achieved: {total_descent:.2f}m\n"
                    f"• Position control: DISABLED\n\n"
                    f"Drone is very low but may not be fully landed. Check visually."
                )
            else:
                status = (
                    f"❌ LANDING INCOMPLETE\n\n"
                    f"• Starting altitude: {start_altitude:.2f}m\n"
                    f"• Current altitude: {final_altitude:.2f}m\n"
                    f"• Descent achieved: {total_descent:.2f}m\n"
                    f"• Position control: DISABLED\n\n"
                    f"Landing may have failed. Drone still airborne at {final_altitude:.2f}m."
                )
            
            return status
        
        @tool
        def get_drone_pose() -> str:
            """
            Get the current position and orientation of the drone.
            
            Use this tool when the user wants to:
            - Know the drone's current position
            - Check where the drone is
            - Get location information
            - Check coordinates
            - See current altitude
            - Get pose data
            
            Returns:
                str: Current drone position (x, y, z) and orientation information
            """
            node.get_logger().info("Getting current drone pose...")
            
            # Get current pose from the node
            current_pose = node.current_pose
            
            # Extract position
            x = current_pose.pose.position.x
            y = current_pose.pose.position.y
            z = current_pose.pose.position.z
            
            # Extract orientation (quaternion)
            qx = current_pose.pose.orientation.x
            qy = current_pose.pose.orientation.y
            qz = current_pose.pose.orientation.z
            qw = current_pose.pose.orientation.w
            
            # Log the pose information
            node.get_logger().info(f"Current position: ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # Format response
            pose_info = (
                f"🚁 Current Drone Pose:\n\n"
                f"📍 Position:\n"
                f"  • X: {x:.2f} m\n"
                f"  • Y: {y:.2f} m\n"
                f"  • Z (Altitude): {z:.2f} m\n\n"
                f"🧭 Orientation (Quaternion):\n"
                f"  • X: {qx:.3f}\n"
                f"  • Y: {qy:.3f}\n"
                f"  • Z: {qz:.3f}\n"
                f"  • W: {qw:.3f}\n\n"
                f"📊 Status: Data retrieved successfully"
            )
            
            return pose_info


        @tool
        def control_gimbal(pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0, reset_to_origin: bool = False) -> str:
            """
            Control the drone's gimbal for camera positioning during SAR operations.
            
            Args:
                pitch: Camera pitch angle in DEGREES (-135 to +45, negative = down)
                roll: Camera roll angle in DEGREES (-45 to +45) 
                yaw: Camera yaw angle in DEGREES (-180 to +180)
                reset_to_origin: If True, resets gimbal to center position (overrides other params)
            
            Returns:
                str: Status of gimbal control command
            """
            # Handle origin reset command
            if reset_to_origin:
                pitch, roll, yaw = 0.0, 0.0, 0.0
                node.get_logger().info("Resetting gimbal to origin position (0°, 0°, 0°)")
            else:
                node.get_logger().info(f"Controlling gimbal: pitch={pitch}°, roll={roll}°, yaw={yaw}°")
            
            # 1. Validate gimbal limits based on SDF
            if not (-135.0 <= pitch <= 45.0):
                return f"Error: Pitch {pitch}° out of range (-135° to +45°)"
            if not (-45.0 <= roll <= 45.0):
                return f"Error: Roll {roll}° out of range (±45°)"
            if not (-180.0 <= yaw <= 180.0):
                return f"Error: Yaw {yaw}° out of practical range (±180°)"
            
            # 2. Create publishers (one-time creation)
            try:
                if not hasattr(node, 'gimbal_pitch_pub'):
                    node.gimbal_pitch_pub = node.create_publisher(
                        Float64, '/drone/gimbal/cmd_pitch', 10)
                    node.get_logger().info("Created gimbal pitch publisher")
                    
                if not hasattr(node, 'gimbal_roll_pub'):
                    node.gimbal_roll_pub = node.create_publisher(
                        Float64, '/drone/gimbal/cmd_roll', 10)
                    node.get_logger().info("Created gimbal roll publisher")
                    
                if not hasattr(node, 'gimbal_yaw_pub'):
                    node.gimbal_yaw_pub = node.create_publisher(
                        Float64, '/drone/gimbal/cmd_yaw', 10)
                    node.get_logger().info("Created gimbal yaw publisher")
            except Exception as e:
                return f"Error creating gimbal publishers: {str(e)}"
            
            # 3. Convert degrees to radians with coordinate system corrections
            pitch_rad = math.radians(-pitch)  # Invert pitch for intuitive up/down
            roll_rad = math.radians(-roll)    # Invert roll for intuitive left/right tilt
            yaw_rad = math.radians(-yaw)      # Invert yaw for intuitive left/right pan
            
            # 4. Create and publish Float64 messages
            try:
                # Pitch command
                pitch_msg = Float64()
                pitch_msg.data = pitch_rad
                node.gimbal_pitch_pub.publish(pitch_msg)
                
                # Roll command  
                roll_msg = Float64()
                roll_msg.data = roll_rad
                node.gimbal_roll_pub.publish(roll_msg)
                
                # Yaw command
                yaw_msg = Float64()
                yaw_msg.data = yaw_rad
                node.gimbal_yaw_pub.publish(yaw_msg)
                
                node.get_logger().info(f"Gimbal commands sent: P={pitch_rad:.3f}rad, R={roll_rad:.3f}rad, Y={yaw_rad:.3f}rad")
                
            except Exception as e:
                return f"Error publishing gimbal commands: {str(e)}"
            
            # 5. Return success status
            if reset_to_origin:
                return (
                    f"✅ Gimbal reset to origin successful!\n\n"
                    f"📐 Position reset:\n"
                    f"  • Pitch: 0.0° (center)\n"
                    f"  • Roll: 0.0° (level)\n"
                    f"  • Yaw: 0.0° (forward)\n\n"
                    f"🎥 Camera is now centered for forward SAR operations."
                )
            else:
                return (
                    f"✅ Gimbal control successful!\n\n"
                    f"📐 Commands sent:\n"
                    f"  • Pitch: {pitch:.1f}° ({pitch_rad:.3f} rad)\n"
                    f"  • Roll: {roll:.1f}° ({roll_rad:.3f} rad)\n"
                    f"  • Yaw: {yaw:.1f}° ({yaw_rad:.3f} rad)\n\n"
                    f"🎥 Camera should now be positioned for SAR operations."
                )
            
        @tool
        def camera_feed(action: str = "start") -> str:
            """
            Control camera feed display in a pop-up window for SAR operations.
            
            Use this tool when the user wants to:
            - Start/open/show camera feed: "show camera", "start camera", "open camera feed"
            - Stop/close camera feed: "close camera", "stop camera", "close camera window"
            
            Args:
                action: "start" to open camera feed, "stop" to close camera feed
            
            Returns:
                str: Status of camera feed operation
            """
            
            if action.lower() == "start":
                # Start camera feed
                node.get_logger().info("Starting SAR camera feed...")
                
                # Initialize camera components if needed
                if not hasattr(node, 'camera_bridge'):
                    node.camera_bridge = CvBridge()
                    node.camera_active = False
                    node.latest_frame = None
                    
                # Create camera subscriber if it doesn't exist
                if not hasattr(node, 'camera_subscriber'):
                    def camera_callback(msg):
                        try:
                            # Convert ROS image to OpenCV
                            cv_image = node.camera_bridge.imgmsg_to_cv2(msg, "bgr8")
                            # Resize to fixed 480x480 resolution
                            resized_image = cv2.resize(cv_image, (480, 480))
                            node.latest_frame = resized_image
                        except Exception as e:
                            node.get_logger().error(f"Camera callback error: {str(e)}")
                    
                    node.camera_subscriber = node.create_subscription(
                        Image, '/drone/gimbal_camera', camera_callback, 10)
                    node.get_logger().info("Created camera subscriber")
                
                # Start camera display thread
                if not node.camera_active:
                    node.camera_active = True
                    
                    def camera_display_thread():
                        cv2.namedWindow("SAR Camera Feed", cv2.WINDOW_NORMAL)
                        cv2.resizeWindow("SAR Camera Feed", 480, 480)
                        
                        while node.camera_active and node.running:
                            if node.latest_frame is not None:
                                # Add timestamp overlay
                                frame_with_overlay = node.latest_frame.copy()
                                timestamp = time.strftime("%H:%M:%S")
                                cv2.putText(frame_with_overlay, f"SAR CAM {timestamp}", 
                                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                
                                cv2.imshow("SAR Camera Feed", frame_with_overlay)
                            
                            # Limit frame rate (30 FPS max)
                            if cv2.waitKey(33) & 0xFF == ord('q'):
                                break
                        
                        cv2.destroyWindow("SAR Camera Feed")
                    
                    node.camera_thread = threading.Thread(target=camera_display_thread)
                    node.camera_thread.daemon = True
                    node.camera_thread.start()
                    
                    return (
                        f"✅ SAR Camera feed started!\n\n"
                        f"📺 Window: 480x480 resolution\n"
                        f"🎥 Live feed from gimbal camera\n"
                        f"⏱️ Timestamp overlay enabled\n\n"
                        f"Press 'q' in camera window to close or use 'stop' command."
                    )
                else:
                    return "Camera feed is already active."
                    
            elif action.lower() == "stop":
                # Stop camera feed
                node.get_logger().info("Stopping SAR camera feed...")
                
                if hasattr(node, 'camera_active') and node.camera_active:
                    node.camera_active = False
                    
                    # Wait for thread to finish
                    if hasattr(node, 'camera_thread') and node.camera_thread.is_alive():
                        node.camera_thread.join(timeout=2.0)
                    
                    # Destroy OpenCV windows
                    cv2.destroyAllWindows()
                    
                    return (
                        f"✅ SAR Camera feed stopped!\n\n"
                        f"📺 Camera window closed\n"
                        f"🎥 Video stream disconnected\n\n"
                        f"Ready for next camera operation."
                    )
                else:
                    return "No active camera feed to stop."
            
            else:
                return f"Error: Invalid action '{action}'. Use 'start' or 'stop'."
        # @tool
        # def drone_status() -> str:
        #     """
        #     Get comprehensive drone status including battery, flight state, position, and system health.
            
        #     Use this tool when the user wants to:
        #     - Check overall drone status
        #     - Get battery information
        #     - Check flight mode and armed status
        #     - Get comprehensive system health
        #     - Monitor operational readiness
        #     - Check all systems at once
            
        #     Returns:
        #         str: Comprehensive drone status report including battery, flight state, position, and system health
        #     """
        #     node.get_logger().info("Getting comprehensive drone status...")
            
        #     status_report = []
        #     status_report.append("🚁 COMPREHENSIVE DRONE STATUS REPORT")
        #     status_report.append("=" * 50)
            
        #     # ====================== BATTERY STATUS ======================
        #     try:
        #         # Create battery subscriber if it doesn't exist
        #         if not hasattr(node, 'battery_data'):
        #             node.battery_data = None
                    
        #             def battery_callback(msg):
        #                 node.battery_data = msg
                    
        #             if not hasattr(node, 'battery_subscriber'):
        #                 node.battery_subscriber = node.create_subscription(
        #                     BatteryState, '/drone/mavros/battery', battery_callback, 10)
        #                 node.get_logger().info("Created battery subscriber")
                        
        #                 # Wait briefly for data
        #                 time.sleep(0.5)
                
        #         status_report.append("\n🔋 BATTERY STATUS:")
        #         if node.battery_data is not None:
        #             voltage = node.battery_data.voltage
        #             percentage = node.battery_data.percentage * 100 if node.battery_data.percentage >= 0 else -1
                    
        #             # Battery health assessment
        #             if percentage > 75:
        #                 battery_health = "🟢 EXCELLENT"
        #             elif percentage > 50:
        #                 battery_health = "🟡 GOOD"
        #             elif percentage > 25:
        #                 battery_health = "🟠 CAUTION"
        #             else:
        #                 battery_health = "🔴 CRITICAL"
                    
        #             status_report.append(f"  • Voltage: {voltage:.2f}V")
        #             if percentage >= 0:
        #                 status_report.append(f"  • Percentage: {percentage:.1f}%")
        #                 status_report.append(f"  • Health: {battery_health}")
        #             else:
        #                 status_report.append(f"  • Percentage: Unknown")
        #                 status_report.append(f"  • Health: 🟡 MONITORING")
        #         else:
        #             status_report.append("  • Status: ⚠️ Battery data not available")
                    
        #     except Exception as e:
        #         status_report.append(f"  • Error: ❌ Battery status unavailable ({str(e)})")
            
        #     # ====================== FLIGHT STATE ======================
        #     try:
        #         # Create state subscriber if it doesn't exist
        #         if not hasattr(node, 'flight_state_data'):
        #             node.flight_state_data = None
                    
        #             def state_callback(msg):
        #                 node.flight_state_data = msg
                    
        #             if not hasattr(node, 'state_subscriber'):
        #                 node.state_subscriber = node.create_subscription(
        #                     State, '/drone/mavros/state', state_callback, 10)
        #                 node.get_logger().info("Created state subscriber")
                        
        #                 # Wait briefly for data
        #                 time.sleep(0.5)
                
        #         status_report.append("\n✈️ FLIGHT STATE:")
        #         if node.flight_state_data is not None:
        #             armed = "🟢 ARMED" if node.flight_state_data.armed else "🔴 DISARMED"
        #             connected = "🟢 CONNECTED" if node.flight_state_data.connected else "🔴 DISCONNECTED"
        #             mode = node.flight_state_data.mode
                    
        #             status_report.append(f"  • Armed Status: {armed}")
        #             status_report.append(f"  • Connection: {connected}")
        #             status_report.append(f"  • Flight Mode: {mode}")
        #             status_report.append(f"  • System ID: {node.flight_state_data.system_status}")
        #         else:
        #             status_report.append("  • Status: ⚠️ Flight state data not available")
                    
        #     except Exception as e:
        #         status_report.append(f"  • Error: ❌ Flight state unavailable ({str(e)})")
            
        #     # ====================== POSITION STATUS ======================
        #     status_report.append("\n📍 POSITION STATUS:")
        #     try:
        #         # Local position (already available from existing subscription)
        #         current_pose = node.current_pose
        #         x = current_pose.pose.position.x
        #         y = current_pose.pose.position.y
        #         z = current_pose.pose.position.z
                
        #         status_report.append(f"  • Local Position:")
        #         status_report.append(f"    - X: {x:.2f} m")
        #         status_report.append(f"    - Y: {y:.2f} m")
        #         status_report.append(f"    - Z (Altitude): {z:.2f} m")
                
        #         # GPS position
        #         if not hasattr(node, 'gps_data'):
        #             node.gps_data = None
                    
        #             def gps_callback(msg):
        #                 node.gps_data = msg
                    
        #             if not hasattr(node, 'gps_subscriber'):
        #                 from sensor_msgs.msg import NavSatFix
        #                 node.gps_subscriber = node.create_subscription(
        #                     NavSatFix, '/drone/mavros/global_position/global', gps_callback, 10)
        #                 node.get_logger().info("Created GPS subscriber")
                        
        #                 # Wait briefly for data
        #                 time.sleep(0.5)
                
        #         if node.gps_data is not None:
        #             lat = node.gps_data.latitude
        #             lon = node.gps_data.longitude
        #             alt = node.gps_data.altitude
                    
        #             # GPS fix quality
        #             if hasattr(node.gps_data, 'status'):
        #                 gps_quality = "🟢 GOOD FIX" if node.gps_data.status.status >= 0 else "🟡 NO FIX"
        #             else:
        #                 gps_quality = "🟡 UNKNOWN"
                    
        #             status_report.append(f"  • GPS Position:")
        #             status_report.append(f"    - Latitude: {lat:.6f}°")
        #             status_report.append(f"    - Longitude: {lon:.6f}°")
        #             status_report.append(f"    - GPS Altitude: {alt:.2f} m")
        #             status_report.append(f"    - GPS Quality: {gps_quality}")
        #         else:
        #             status_report.append(f"  • GPS: ⚠️ GPS data not available")
                    
        #     except Exception as e:
        #         status_report.append(f"  • Error: ❌ Position data error ({str(e)})")
            
        #     # ====================== FLIGHT PERFORMANCE ======================
        #     try:
        #         # Create VFR HUD subscriber if it doesn't exist
        #         if not hasattr(node, 'vfr_data'):
        #             node.vfr_data = None
                    
        #             def vfr_callback(msg):
        #                 node.vfr_data = msg
                    
        #             if not hasattr(node, 'vfr_subscriber'):
        #                 node.vfr_subscriber = node.create_subscription(
        #                     VfrHud, '/drone/mavros/vfr_hud', vfr_callback, 10)
        #                 node.get_logger().info("Created VFR HUD subscriber")
                        
        #                 # Wait briefly for data
        #                 time.sleep(0.5)
                
        #         status_report.append("\n📊 FLIGHT PERFORMANCE:")
        #         if node.vfr_data is not None:
        #             airspeed = node.vfr_data.airspeed
        #             groundspeed = node.vfr_data.groundspeed
        #             climb = node.vfr_data.climb
        #             throttle = node.vfr_data.throttle
                    
        #             status_report.append(f"  • Airspeed: {airspeed:.1f} m/s")
        #             status_report.append(f"  • Ground Speed: {groundspeed:.1f} m/s")
        #             status_report.append(f"  • Climb Rate: {climb:.1f} m/s")
        #             status_report.append(f"  • Throttle: {throttle:.0f}%")
        #         else:
        #             status_report.append("  • Status: ⚠️ Flight performance data not available")
                    
        #     except Exception as e:
        #         status_report.append(f"  • Error: ❌ Performance data error ({str(e)})")
            
        #     # ====================== SYSTEM SUMMARY ======================
        #     status_report.append("\n🎯 OPERATIONAL READINESS:")
            
        #     # Determine overall system status
        #     try:
        #         ready_for_mission = True
        #         warnings = []
                
        #         # Check battery
        #         if hasattr(node, 'battery_data') and node.battery_data is not None:
        #             if hasattr(node.battery_data, 'percentage') and node.battery_data.percentage >= 0:
        #                 battery_pct = node.battery_data.percentage * 100
        #                 if battery_pct < 25:
        #                     ready_for_mission = False
        #                     warnings.append("🔴 CRITICAL: Low battery")
        #                 elif battery_pct < 50:
        #                     warnings.append("🟡 CAUTION: Battery below 50%")
                
        #         # Check connection
        #         if hasattr(node, 'flight_state_data') and node.flight_state_data is not None:
        #             if not node.flight_state_data.connected:
        #                 ready_for_mission = False
        #                 warnings.append("🔴 CRITICAL: Not connected to autopilot")
                
        #         # Check position data
        #         if not hasattr(node, 'current_pose') or node.current_pose is None:
        #             warnings.append("🟡 CAUTION: Position data may be unreliable")
                
        #         # Overall status
        #         if ready_for_mission and len(warnings) == 0:
        #             status_report.append("  • Overall Status: 🟢 READY FOR MISSION")
        #         elif ready_for_mission:
        #             status_report.append("  • Overall Status: 🟡 OPERATIONAL WITH CAUTIONS")
        #         else:
        #             status_report.append("  • Overall Status: 🔴 NOT READY - ISSUES DETECTED")
                
        #         # Add warnings
        #         if warnings:
        #             status_report.append("  • Warnings:")
        #             for warning in warnings:
        #                 status_report.append(f"    - {warning}")
                        
        #     except Exception as e:
        #         status_report.append(f"  • Error: ❌ Cannot determine readiness ({str(e)})")
                
        #     # ====================== TIMESTAMP ======================
        #     from datetime import datetime
        #     timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        #     status_report.append(f"\n⏰ Report generated: {timestamp}")
        #     status_report.append("=" * 50)
            
        #     # Join all status lines
        #     final_report = "\n".join(status_report)
            
        #     # Log the status check
        #     node.get_logger().info("Comprehensive status check completed")
            
        #     return final_report
        
        @tool
        def go_to_position(x: float, y: float, z: float) -> str:
            """
            Move the drone to a specific position (x, y, z) in local coordinates.
            
            Args:
                x: Target X position in meters
                y: Target Y position in meters  
                z: Target Z position (altitude) in meters
            
            Returns:
                str: Status message about the movement command
            """
            # Input validation
            if z <= 0:
                return "Error: Altitude (z) must be positive"
            if z > 20:
                return "Error: Altitude exceeds safe limit (20m max)"
            
            # Get current position
            current_x = node.current_pose.pose.position.x
            current_y = node.current_pose.pose.position.y
            current_z = node.current_pose.pose.position.z
            
            node.get_logger().info(f"Moving from ({current_x:.2f}, {current_y:.2f}, {current_z:.2f}) to ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # Create setpoint publisher if it doesn't exist
            if not hasattr(node, 'setpoint_pub'):
                node.setpoint_pub = node.create_publisher(
                    PoseStamped,
                    '/drone/mavros/setpoint_position/local',
                    10
                )
                node.get_logger().info("Created setpoint publisher")
            
            # Create and publish new setpoint
            setpoint = PoseStamped()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = x
            setpoint.pose.position.y = y
            setpoint.pose.position.z = z
            setpoint.pose.orientation.w = 1.0
            
            # Store setpoint and enable publishing
            node.target_setpoint = setpoint
            node.publish_setpoints = True
            
            # Start setpoint publisher thread if not running
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                def setpoint_publisher():
                    rate = node.create_rate(10)
                    while node.running:
                        if getattr(node, 'publish_setpoints', False) and hasattr(node, 'target_setpoint'):
                            node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                            node.setpoint_pub.publish(node.target_setpoint)
                        rate.sleep()
                
                node.setpoint_thread = threading.Thread(target=setpoint_publisher)
                node.setpoint_thread.daemon = True
                node.setpoint_thread.start()
                node.get_logger().info("Started setpoint publisher thread")
            
            return f"✅ Moving to position ({x:.2f}, {y:.2f}, {z:.2f})\n• Target set successfully\n• Publishing setpoints: ✓"

################################################################################################################################################################################################################################

        @tool
        def analyze_drone_camera() -> str:
            """
            Analyze current drone camera feed using vision AI specialist.
            Captures current frame and sends to qwen2.5vl:7b for SAR analysis.
            
            Use this tool when user wants to:
            - Analyze what the drone camera sees
            - Look for humans in the camera feed
            - Assess survivor conditions
            - Get detailed scene analysis for SAR operations
            
            Returns:
                str: Detailed analysis of humans and their conditions in the scene
            """
            # 1. Check if camera feed is active
            if not hasattr(node, 'camera_active') or not node.camera_active:
                return "❌ Camera feed not active. Please start camera feed first using 'show camera' or 'camera_feed start'."
            
            # 2. Check if current frame is available
            if not hasattr(node, 'latest_frame') or node.latest_frame is None:
                return "❌ No camera frame available. Please ensure camera feed is working properly."
            
            # 3. Get SAR analysis prompt from prompts module
            sar_prompt = SARAnalysisPrompts.human_condition_analysis()

            node.get_logger().info("Starting drone camera analysis with vision specialist...")
            
            # 4. Call vision specialist
            try:
                analysis_result = call_vision_specialist(
                    cv_image=node.latest_frame,
                    prompt=sar_prompt,
                    model_name="qwen2.5vl:7b",
                    node=node
                )
                
                return f"🔍 DRONE CAMERA ANALYSIS:\n\n{analysis_result}"
                
            except Exception as e:
                node.get_logger().error(f"Vision analysis failed: {str(e)}")
                return f"❌ Vision analysis failed: {str(e)}"
                       
        # Return the tools
        return [
            takeoff,
            land,
            get_drone_pose,
            control_gimbal,
            camera_feed,
            # drone_status,
            go_to_position,
            analyze_drone_camera
        ]