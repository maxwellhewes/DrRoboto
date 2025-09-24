#!/usr/bin/env python3
"""
ROS Controller Module
Handles ROS 2 communication for the robotic arm
"""

import os
import time
import math
import threading
import logging
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

logger = logging.getLogger(__name__)

try:
    from .config import ROS_TOPIC_PREFIX
except ImportError:
    # Fallback for direct execution
    ROS_TOPIC_PREFIX = os.getenv('ROS_TOPIC_PREFIX', '')

# Global variables for ROS state
ros_node = None
last_status = "Ready"
last_joint_state = None

class RobotControllerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 node for web-based robot control.
        
        Sets up publishers for joint states and pose commands, and a subscriber
        for robot status updates. This node acts as a bridge between the web
        interface and the robot simulation.
        """
        super().__init__('web_robot_controller')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, f'{ROS_TOPIC_PREFIX}/joint_states', 10)
        self.pose_pub = self.create_publisher(String, f'{ROS_TOPIC_PREFIX}/arm_pose_command', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            f'{ROS_TOPIC_PREFIX}/arm_status',
            self.status_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            f'{ROS_TOPIC_PREFIX}/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Web Robot Controller node initialized')
    
    def status_callback(self, msg):
        """Handle incoming robot status messages.
        
        Args:
            msg (String): ROS message containing the current robot status
        """
        global last_status
        last_status = msg.data
        self.get_logger().info(f'Robot status: {msg.data}')
    
    def joint_state_callback(self, msg):
        """Handle incoming joint state messages and broadcast via WebSocket.
        
        Args:
            msg (JointState): ROS message containing current joint positions
        """
        global last_joint_state
        last_joint_state = msg
        
        # Extract joint positions and convert to degrees
        joint_data = {
            'joint0': {
                'position': msg.position[0] if len(msg.position) > 0 else 0.0,
                'position_degrees': math.degrees(msg.position[0]) if len(msg.position) > 0 else 0.0,
                'velocity': msg.velocity[0] if len(msg.velocity) > 0 else 0.0,
                'effort': msg.effort[0] if len(msg.effort) > 0 else 0.0
            },
            'joint1': {
                'position': msg.position[1] if len(msg.position) > 1 else 0.0,
                'position_degrees': math.degrees(msg.position[1]) if len(msg.position) > 1 else 0.0,
                'velocity': msg.velocity[1] if len(msg.velocity) > 1 else 0.0,
                'effort': msg.effort[1] if len(msg.effort) > 1 else 0.0
            },
            'joint2': {
                'position': msg.position[2] if len(msg.position) > 2 else 0.0,
                'position_degrees': math.degrees(msg.position[2]) if len(msg.position) > 2 else 0.0,
                'velocity': msg.velocity[2] if len(msg.velocity) > 2 else 0.0,
                'effort': msg.effort[2] if len(msg.effort) > 2 else 0.0
            },
            'timestamp': time.time()
        }
        
        # Broadcast joint state via WebSocket
        # Note: This will be set by the main app to avoid circular imports
        if hasattr(self, '_socketio'):
            self._socketio.emit('joint_state_update', joint_data)
        self.get_logger().debug(f'Broadcasted joint state: {joint_data}')
    
    def publish_joint_state(self, joint0, joint1, joint2):
        """Publish joint state directly to control robot arm position.
        
        Args:
            joint0 (float): Position of joint 0 in radians
            joint1 (float): Position of joint 1 in radians
            joint2 (float): Position of joint 2 in radians
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint0', 'joint1', 'joint2']
        joint_state.position = [joint0, joint1, joint2]
        joint_state.velocity = [0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0]
        
        self.joint_pub.publish(joint_state)
        self.get_logger().info(f'Published joint state: joint0={joint0:.3f}, joint1={joint1:.3f}, joint2={joint2:.3f}')
    
    def publish_pose_command(self, pose_name):
        """Publish a named pose command to the robot.
        
        Args:
            pose_name (str): Name of the predefined pose to execute
        """
        pose_msg = String()
        pose_msg.data = pose_name
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published pose command: {pose_name}')

def init_ros():
    """Initialize ROS 2 node in a separate thread.
    
    Sets up the ROS 2 node and starts a background thread to handle
    ROS communication. This allows the Flask app to run concurrently
    with ROS 2 message processing.
    
    Returns:
        bool: True if initialization successful, False otherwise
    """
    global ros_node
    
    logger.info("Starting ROS 2 initialization...")
    
    try:
        logger.info("Initializing rclpy...")
        rclpy.init()
        logger.info("rclpy initialized successfully")
        
        logger.info("Creating RobotControllerNode...")
        ros_node = RobotControllerNode()
        logger.info("RobotControllerNode created successfully")
        
        # Spin the node in a separate thread
        def spin_node():
            logger.info("Starting ROS 2 spin thread...")
            while rclpy.ok():
                try:
                    rclpy.spin_once(ros_node, timeout_sec=0.1)
                except Exception as e:
                    logger.error(f"Error in ROS 2 spin: {e}")
                    break
            logger.info("ROS 2 spin thread ended")
        
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        logger.info("ROS 2 spin thread started")
        
        logger.info("ROS 2 node initialized successfully")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize ROS 2: {e}")
        logger.error(f"Exception type: {type(e).__name__}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        return False

def get_robot_status():
    """Get current robot status from the last received status message.
    
    Returns:
        str: The most recent robot status message
    """
    global last_status
    return last_status

def get_ros_node():
    """Get the current ROS node instance.
    
    Returns:
        RobotControllerNode or None: The ROS node if initialized, None otherwise
    """
    global ros_node
    return ros_node

def get_last_joint_state():
    """Get the last received joint state.
    
    Returns:
        JointState or None: The last joint state message if available, None otherwise
    """
    global last_joint_state
    return last_joint_state
