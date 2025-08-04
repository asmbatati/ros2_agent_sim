#!/usr/bin/env python3
"""
Specialist LLM models for multi-agent AI ecosystem.
This module provides utility functions for calling specialized models.
"""

import cv2
import base64
import logging
from langchain_core.messages import HumanMessage
from langchain_ollama import ChatOllama

logger = logging.getLogger(__name__)


def opencv_to_base64(cv_image, resize_dimensions=(640, 480)):
    """
    Convert OpenCV image to base64 string for vision models.
    Includes automatic resizing for optimal processing.
    
    Args:
        cv_image: OpenCV image (numpy array)
        resize_dimensions: Target size (width, height). Default: (640, 480)
        
    Returns:
        str: Base64 encoded JPEG image
    """
    # 1. Resize image 
    resized_image = cv2.resize(cv_image, resize_dimensions)
    
    # 2. Encode image to JPEG format (85% quality for balance)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
    _, buffer = cv2.imencode('.jpg', resized_image, encode_param)
    
    # 3. Convert to base64
    base64_image = base64.b64encode(buffer).decode('utf-8')
    
    return base64_image


def call_vision_specialist(
    cv_image,                              
    prompt: str,                           
    model_name: str = "qwen2.5vl:7b",      
    node=None                              
) -> str:
    """
    Call local vision specialist using langchain_ollama.
    
    Args:
        cv_image: OpenCV image (numpy array)
        prompt: Task-specific prompt for analysis
        model_name: Vision model to use
        node: ROS node for logging (optional)
        
    Returns:
        str: Specialist model response
    """
    try:
        # 1. Convert and resize image to base64 (now combined)
        base64_image = opencv_to_base64(cv_image)
        
        # 2. Create vision LLM instance
        vision_llm = ChatOllama(
            model=model_name,
            temperature=0.0,
            max_retries=3,  
            num_ctx=8192,
        )
        
        # 3. Create vision message
        message = HumanMessage(content=[
            {"type": "image_url", "image_url": f"data:image/jpeg;base64,{base64_image}"},
            {"type": "text", "text": prompt}
        ])
        
        # 4. Call vision model  
        response = vision_llm.invoke([message])
        
        if node:
            node.get_logger().info(f"Vision specialist ({model_name}) completed analysis")
            
        return response.content
        
    except Exception as e:
        error_msg = f"Vision specialist error: {str(e)}"
        if node:
            node.get_logger().error(error_msg)
        return error_msg


# ================================ FUTURE SPECIALIST FUNCTIONS ================================

def call_navigation_specialist(
    current_position: dict,
    target_position: dict,
    obstacles: list = None,
    model_name: str = "qwen3:8b",
    node=None
) -> str:
    """
    TODO: Call navigation planning specialist for optimal path planning.
    
    Args:
        current_position: Current robot position {x, y, z}
        target_position: Target position {x, y, z}
        obstacles: List of known obstacles
        model_name: Navigation specialist model
        node: ROS node for logging
        
    Returns:
        str: Navigation plan and waypoints
    """
    # TODO: Implement navigation specialist
    return "Navigation specialist not implemented yet"


def call_safety_specialist(
    robot_status: dict,
    environment_data: dict = None,
    model_name: str = "qwen3:8b",
    node=None
) -> str:
    """
    TODO: Call safety assessment specialist for risk evaluation.
    
    Args:
        robot_status: Current robot status (battery, sensors, etc.)
        environment_data: Environmental conditions
        model_name: Safety specialist model
        node: ROS node for logging
        
    Returns:
        str: Safety assessment and recommendations
    """
    # TODO: Implement safety specialist
    return "Safety specialist not implemented yet"


def call_diagnostic_specialist(
    system_logs: list,
    error_messages: list = None,
    model_name: str = "qwen3:8b",
    node=None
) -> str:
    """
    TODO: Call technical diagnostic specialist for system troubleshooting.
    
    Args:
        system_logs: Recent system logs
        error_messages: List of error messages
        model_name: Diagnostic specialist model
        node: ROS node for logging
        
    Returns:
        str: Diagnostic analysis and solutions
    """
    # TODO: Implement diagnostic specialist
    return "Diagnostic specialist not implemented yet"


def call_mission_planning_specialist(
    mission_objectives: list,
    available_resources: dict,
    time_constraints: dict = None,
    model_name: str = "qwen3:8b",
    node=None
) -> str:
    """
    TODO: Call mission planning specialist for SAR operation optimization.
    
    Args:
        mission_objectives: List of mission goals
        available_resources: Available robots and equipment
        time_constraints: Time limits and priorities
        model_name: Mission planning specialist model
        node: ROS node for logging
        
    Returns:
        str: Optimized mission plan
    """
    # TODO: Implement mission planning specialist
    return "Mission planning specialist not implemented yet"


# Export functions for use in tools
__all__ = [
    'call_vision_specialist', 
    'opencv_to_base64',
    'call_navigation_specialist',
    'call_safety_specialist', 
    'call_diagnostic_specialist',
    'call_mission_planning_specialist'
]