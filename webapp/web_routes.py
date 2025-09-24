#!/usr/bin/env python3
"""
Web Routes Module
Contains Flask API endpoints for the DrRoboto web application
"""

import os
import math
import time
import logging
from flask import render_template, request, jsonify

try:
    from .ros_controller import get_ros_node, get_robot_status
    from .config import HOME_POSE, POSES, ROS_TOPIC_PREFIX
except ImportError:
    from ros_controller import get_ros_node, get_robot_status
    from config import HOME_POSE, POSES, ROS_TOPIC_PREFIX

logger = logging.getLogger(__name__)

def register_routes(app):
    """Register all Flask routes with the app.
    
    Args:
        app: Flask application instance
    """
    
    @app.route('/')
    def index():
        """Render the main control interface page.
        
        Serves the main HTML template with available robot poses.
        
        Returns:
            str: Rendered HTML template or error message
        """
        logger.info("Index page requested")
        try:
            return render_template('index.html')
        except Exception as e:
            logger.error(f"Error rendering index template: {e}")
            return f"Error loading page: {e}", 500

    @app.route('/api/home', methods=['POST'])
    def move_to_home():
        """Move robot to home position.
        
        Returns:
            dict: JSON response indicating success or failure
        """
        ros_node = get_ros_node()
        if not ros_node:
            return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
        
        try:
            # Publish joint state for home position
            joint0, joint1, joint2 = HOME_POSE
            ros_node.publish_joint_state(joint0, joint1, joint2)
            
            return jsonify({
                "success": True, 
                "message": "Moved to home position",
                "pose": HOME_POSE
            })
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route('/api/joint0', methods=['POST'])
    def set_joint0():
        """Set joint 0 angle directly.
        
        Returns:
            dict: JSON response indicating success or failure
        """
        ros_node = get_ros_node()
        if not ros_node:
            return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
        
        try:
            data = request.get_json()
            joint0_angle = data.get('angle', 0.0)
            
            # Validate angle is within limits
            if joint0_angle < -1.57 or joint0_angle > 1.57:
                return jsonify({'success': False, 'error': 'Joint 0 angle out of range (-90° to +90°)'}), 400
            
            # Publish joint state with only joint 0 changed
            # We need to get current joint states for joints 1 and 2
            # For now, we'll use the home pose values
            _, joint1, joint2 = HOME_POSE
            ros_node.publish_joint_state(joint0_angle, joint1, joint2)
            
            return jsonify({
                "success": True, 
                "message": f"Set joint 0 to {joint0_angle:.3f} radians",
                "angle": joint0_angle
            })
        except Exception as e:
            logger.error(f"Error setting joint 0: {e}")
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route('/api/end_effector', methods=['POST'])
    def set_end_effector_position():
        """Set end-effector position using inverse kinematics.
        
        Expects JSON data with 'x', 'y', 'z' values in meters.
        
        Returns:
            dict: JSON response indicating success or failure with joint values
        """
        data = request.get_json()
        logger.info(f"Set end-effector position: {data}")
        if not data or 'x' not in data or 'y' not in data or 'z' not in data:
            return jsonify({"success": False, "error": "Missing x, y, or z parameters"}), 400
        
        ros_node = get_ros_node()
        if not ros_node:
            return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
            
        try:
            x = float(data['x'])
            y = float(data['y'])
            z = float(data['z'])
            
            # Calculate inverse kinematics
            logger.info(f"Calculating IK for position: x={x}, y={y}, z={z}")
            ik_result = inverse_kinematics(x, y, z)
            if ik_result is None:
                logger.warning(f"Position {x}, {y}, {z} is unreachable")
                return jsonify({"success": False, "error": "Target position is unreachable"}), 400
                
            joint0, joint1, joint2 = ik_result
            logger.info(f"IK result: joint0={joint0:.3f}, joint1={joint1:.3f}, joint2={joint2:.3f}")
            
            # Convert to degrees for display
            joint0_deg = math.degrees(joint0)
            joint1_deg = math.degrees(joint1)
            joint2_deg = math.degrees(joint2)
            
            logger.info(f"Publishing joint state: {joint0}, {joint1}, {joint2}")
            
            # Publish joint state directly
            ros_node.publish_joint_state(joint0, joint1, joint2)
            
            return jsonify({
                "success": True,
                "message": f"Set end-effector to: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m",
                "joints": [joint0, joint1, joint2],
                "position": {"x": x, "y": y, "z": z}
            })
                
        except ValueError:
            return jsonify({"success": False, "error": "Invalid position values"}), 400
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route('/api/workspace', methods=['GET'])
    def get_workspace():
        """Get robot workspace bounds for visualization.
        
        Returns:
            dict: JSON response with workspace data
        """
        try:
            workspace = calculate_workspace_bounds()
            return jsonify({
                "success": True,
                "workspace": workspace
            })
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route('/api/joints', methods=['POST'])
    def set_joint_positions():
        """Set specific joint positions for the robot arm.
        
        Expects JSON data with 'joint0', 'joint1' and 'joint2' values in radians.
        
        Returns:
            dict: JSON response indicating success or failure with joint values
        """
        data = request.get_json()
        logger.info(f"Set joint positions: {data}")
        if not data or 'joint0' not in data or 'joint1' not in data or 'joint2' not in data:
            return jsonify({"success": False, "error": "Missing joint0, joint1 or joint2 parameters"}), 400
        
        ros_node = get_ros_node()
        if not ros_node:
            return jsonify({"success": False, "error": "ROS 2 not initialized"}), 500
            
        try:
            joint0 = float(data['joint0'])
            joint1 = float(data['joint1'])
            joint2 = float(data['joint2'])
            
            # Convert to degrees for display
            joint0_deg = math.degrees(joint0)
            joint1_deg = math.degrees(joint1)
            joint2_deg = math.degrees(joint2)
            
            logger.info(f"Publishing joint state: {joint0}, {joint1}, {joint2}")
            
            # Publish joint state directly
            ros_node.publish_joint_state(joint0, joint1, joint2)
            
            return jsonify({
                "success": True,
                "message": f"Set joints to: joint0={joint0_deg:.1f}°, joint1={joint1_deg:.1f}°, joint2={joint2_deg:.1f}°",
                "joints": [joint0, joint1, joint2]
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
        ros_node = get_ros_node()
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
        ros_node = get_ros_node()
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
        ros_node = get_ros_node()
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
                    ros_node.publish_joint_state(joint_positions[0], joint_positions[1], joint_positions[2])
                
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
        ros_node = get_ros_node()
        return jsonify({
            "ros_initialized": ros_node is not None,
            "ros_topics": [
                f'{ROS_TOPIC_PREFIX}/joint_states',
                f'{ROS_TOPIC_PREFIX}/arm_pose_command',
                f'{ROS_TOPIC_PREFIX}/arm_status'
            ],
            "poses_available": list(POSES.keys()),
            "last_status": get_robot_status(),
            "environment": {
                "ROS_TOPIC_PREFIX": ROS_TOPIC_PREFIX,
                "FLASK_ENV": os.getenv('FLASK_ENV', 'not set')
            }
        })
