#!/usr/bin/env python3
"""
DrRoboto Web Control Interface
Flask web application for controlling the robotic arm simulation
Uses direct ROS 2 network communication instead of Docker exec
"""

import os
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import threading
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Enable Flask debug logging
app.logger.setLevel(logging.DEBUG)

# Configuration
ROS_TOPIC_PREFIX = os.getenv('ROS_TOPIC_PREFIX', '')

# Predefined poses (joint1, joint2) in radians
POSES = {
    'home': [0.0, 0.0],                    # Straight up
    'pose1': [math.pi/4, -math.pi/4],     # 45 degrees, -45 degrees
    'pose2': [-math.pi/4, math.pi/3],     # -45 degrees, 60 degrees
    'pose3': [math.pi/2, -math.pi/6],     # 90 degrees, -30 degrees
    'wave': [math.pi/3, -math.pi/3],      # Custom wave pose
    'point': [math.pi/6, math.pi/4]       # Custom pointing pose
}

# Global ROS 2 node and publishers
ros_node = None
joint_pub = None
pose_pub = None
status_sub = None
joint_sub = None
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
            'joint1': {
                'position': msg.position[0] if len(msg.position) > 0 else 0.0,
                'position_degrees': math.degrees(msg.position[0]) if len(msg.position) > 0 else 0.0,
                'velocity': msg.velocity[0] if len(msg.velocity) > 0 else 0.0,
                'effort': msg.effort[0] if len(msg.effort) > 0 else 0.0
            },
            'joint2': {
                'position': msg.position[1] if len(msg.position) > 1 else 0.0,
                'position_degrees': math.degrees(msg.position[1]) if len(msg.position) > 1 else 0.0,
                'velocity': msg.velocity[1] if len(msg.velocity) > 1 else 0.0,
                'effort': msg.effort[1] if len(msg.effort) > 1 else 0.0
            },
            'timestamp': time.time()
        }
        
        # Broadcast joint state via WebSocket
        socketio.emit('joint_state_update', joint_data)
        self.get_logger().debug(f'Broadcasted joint state: {joint_data}')
    
    def publish_joint_state(self, joint1, joint2):
        """Publish joint state directly to control robot arm position.
        
        Args:
            joint1 (float): Position of joint 1 in radians
            joint2 (float): Position of joint 2 in radians
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2']
        joint_state.position = [joint1, joint2]
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_state)
        self.get_logger().info(f'Published joint state: joint1={joint1:.3f}, joint2={joint2:.3f}')
    
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
    global ros_node, joint_pub, pose_pub, status_sub, joint_sub
    
    logger.info("Starting ROS 2 initialization...")
    
    try:
        logger.info("Initializing rclpy...")
        rclpy.init()
        logger.info("rclpy initialized successfully")
        
        logger.info("Creating RobotControllerNode...")
        ros_node = RobotControllerNode()
        joint_pub = ros_node.joint_pub
        pose_pub = ros_node.pose_pub
        status_sub = ros_node.status_sub
        joint_sub = ros_node.joint_sub
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

@app.route('/')
def index():
    """Render the main control interface page.
    
    Serves the main HTML template with available robot poses.
    
    Returns:
        str: Rendered HTML template or error message
    """
    logger.info("Index page requested")
    try:
        return render_template('index.html', poses=POSES)
    except Exception as e:
        logger.error(f"Error rendering index template: {e}")
        return f"Error loading page: {e}", 500

@app.route('/api/poses', methods=['GET'])
def get_poses():
    """Get available robot poses in both radians and degrees.
    
    Returns:
        dict: JSON response containing all predefined poses with
              both radian and degree values for each joint
    """
    poses_with_degrees = {}
    for name, angles in POSES.items():
        poses_with_degrees[name] = {
            'radians': angles,
            'degrees': [math.degrees(angle) for angle in angles]
        }
    return jsonify(poses_with_degrees)

@app.route('/api/move/<pose_name>', methods=['POST'])
def move_to_pose(pose_name):
    """Move robot to a predefined pose.
    
    Args:
        pose_name (str): Name of the pose to move to
    
    Returns:
        dict: JSON response indicating success or failure
    """
    if pose_name not in POSES:
        return jsonify({"success": False, "error": f"Unknown pose: {pose_name}"}), 400
    
    if not ros_node:
        return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
    
    try:
        # Publish pose command
        ros_node.publish_pose_command(pose_name)
        
        # Also publish joint state for direct control
        joint_positions = POSES[pose_name]
        ros_node.publish_joint_state(joint_positions[0], joint_positions[1])
        
        return jsonify({
            "success": True, 
            "message": f"Moved to pose: {pose_name}",
            "pose": POSES[pose_name]
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/api/joints', methods=['POST'])
def set_joint_positions():
    """Set specific joint positions for the robot arm.
    
    Expects JSON data with 'joint1' and 'joint2' values in radians.
    
    Returns:
        dict: JSON response indicating success or failure with joint values
    """
    data = request.get_json()
    logger.info(f"Set joint positions: {data}")
    if not data or 'joint1' not in data or 'joint2' not in data:
        return jsonify({"success": False, "error": "Missing joint1 or joint2 parameters"}), 400
    if not ros_node:
        return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
        
    try:
        joint1 = float(data['joint1'])
        joint2 = float(data['joint2'])
        
        # Convert to degrees for display
        joint1_deg = math.degrees(joint1)
        joint2_deg = math.degrees(joint2)
        
        logger.info(f"Publishing joint state: {joint1}, {joint2}")
        
        # Publish joint state directly
        ros_node.publish_joint_state(joint1, joint2)
        
        return jsonify({
            "success": True,
            "message": f"Set joints to: joint1={joint1_deg:.1f}°, joint2={joint2_deg:.1f}°",
            "joints": [joint1, joint2]
        })
            
    except ValueError:
        return jsonify({"success": False, "error": "Invalid joint values"}), 400
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get current robot status.
    
    Returns:
        dict: JSON response with current robot status or error message
    """
    if not ros_node:
        return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
    
    status = get_robot_status()
    return jsonify({"success": True, "status": status})

@app.route('/api/topics', methods=['GET'])
def get_topics():
    """Get available ROS topics used by the web interface.
    
    Returns:
        dict: JSON response with list of ROS topics for joint states,
              pose commands, and status updates
    """
    if not ros_node:
        return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
    
    # Return known topics that we're using
    topics = [
        f'{ROS_TOPIC_PREFIX}/joint_states',
        f'{ROS_TOPIC_PREFIX}/arm_pose_command',
        f'{ROS_TOPIC_PREFIX}/arm_status'
    ]
    return jsonify({"success": True, "topics": topics})

@app.route('/api/wave', methods=['POST'])
def wave_gesture():
    """Perform a waving gesture sequence.
    
    Executes a predefined sequence of poses to create a waving motion.
    
    Returns:
        dict: JSON response indicating success or failure of the gesture
    """
    if not ros_node:
        return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
    
    wave_sequence = ['home', 'pose1', 'pose2', 'pose1', 'pose2', 'home']
    
    try:
        for i, pose in enumerate(wave_sequence):
            # Publish pose command
            ros_node.publish_pose_command(pose)
            
            # Also publish joint state for direct control
            if pose in POSES:
                joint_positions = POSES[pose]
                ros_node.publish_joint_state(joint_positions[0], joint_positions[1])
            
            # Small delay between poses
            time.sleep(0.5)
        
        return jsonify({
            "success": True,
            "message": "Wave gesture completed",
            "steps": len(wave_sequence)
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint for service monitoring.
    
    Returns:
        dict: JSON response indicating the service is healthy
    """
    return jsonify({"status": "healthy", "service": "drroboto-web"})

@app.route('/debug')
def debug_info():
    """Debug information endpoint for troubleshooting.
    
    Provides detailed information about the application state including
    ROS initialization status, available topics, poses, and environment variables.
    
    Returns:
        dict: JSON response with debug information
    """
    return jsonify({
        "ros_initialized": ros_node is not None,
        "ros_topics": [
            f'{ROS_TOPIC_PREFIX}/joint_states',
            f'{ROS_TOPIC_PREFIX}/arm_pose_command',
            f'{ROS_TOPIC_PREFIX}/arm_status'
        ],
        "poses_available": list(POSES.keys()),
        "last_status": last_status,
        "environment": {
            "ROS_TOPIC_PREFIX": ROS_TOPIC_PREFIX,
            "FLASK_ENV": os.getenv('FLASK_ENV', 'not set')
        }
    })

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    """Handle WebSocket client connection."""
    logger.info(f'Client connected: {request.sid}')
    emit('connection_status', {'status': 'connected', 'message': 'Connected to DrRoboto WebSocket'})
    
    # Send current joint state if available
    if last_joint_state:
        joint_data = {
            'joint1': {
                'position': last_joint_state.position[0] if len(last_joint_state.position) > 0 else 0.0,
                'position_degrees': math.degrees(last_joint_state.position[0]) if len(last_joint_state.position) > 0 else 0.0,
                'velocity': last_joint_state.velocity[0] if len(last_joint_state.velocity) > 0 else 0.0,
                'effort': last_joint_state.effort[0] if len(last_joint_state.effort) > 0 else 0.0
            },
            'joint2': {
                'position': last_joint_state.position[1] if len(last_joint_state.position) > 1 else 0.0,
                'position_degrees': math.degrees(last_joint_state.position[1]) if len(last_joint_state.position) > 1 else 0.0,
                'velocity': last_joint_state.velocity[1] if len(last_joint_state.velocity) > 1 else 0.0,
                'effort': last_joint_state.effort[1] if len(last_joint_state.effort) > 1 else 0.0
            },
            'timestamp': time.time()
        }
        emit('joint_state_update', joint_data)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket client disconnection."""
    logger.info(f'Client disconnected: {request.sid}')

@socketio.on('request_joint_update')
def handle_joint_update_request():
    """Handle request for current joint state."""
    if last_joint_state:
        joint_data = {
            'joint1': {
                'position': last_joint_state.position[0] if len(last_joint_state.position) > 0 else 0.0,
                'position_degrees': math.degrees(last_joint_state.position[0]) if len(last_joint_state.position) > 0 else 0.0,
                'velocity': last_joint_state.velocity[0] if len(last_joint_state.velocity) > 0 else 0.0,
                'effort': last_joint_state.effort[0] if len(last_joint_state.effort) > 0 else 0.0
            },
            'joint2': {
                'position': last_joint_state.position[1] if len(last_joint_state.position) > 1 else 0.0,
                'position_degrees': math.degrees(last_joint_state.position[1]) if len(last_joint_state.position) > 1 else 0.0,
                'velocity': last_joint_state.velocity[1] if len(last_joint_state.velocity) > 1 else 0.0,
                'effort': last_joint_state.effort[1] if len(last_joint_state.effort) > 1 else 0.0
            },
            'timestamp': time.time()
        }
        emit('joint_state_update', joint_data)
    else:
        emit('joint_state_update', {'error': 'No joint state data available'})

if __name__ == '__main__':
    logger.info("Starting DrRoboto Web Application...")
    logger.info(f"Environment variables: ROS_TOPIC_PREFIX='{ROS_TOPIC_PREFIX}'")
    
    # Initialize ROS 2
    logger.info("Attempting to initialize ROS 2...")
    if init_ros():
        logger.info("Starting Flask-SocketIO app with ROS 2 support...")
        socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
    else:
        logger.warning("Failed to initialize ROS 2. Starting Flask-SocketIO app without ROS 2 support...")
        socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
