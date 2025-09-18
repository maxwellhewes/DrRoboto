#!/usr/bin/env python3
"""
WebSocket Handlers Module
Contains Socket.IO event handlers for real-time communication
"""

import math
import time
import logging
from flask import request
from flask_socketio import emit

try:
    from .ros_controller import get_ros_node, get_last_joint_state
    from .config import HOME_POSE
except ImportError:
    from ros_controller import get_ros_node, get_last_joint_state
    from config import HOME_POSE

logger = logging.getLogger(__name__)

def register_websocket_handlers(socketio):
    """Register all Socket.IO event handlers.
    
    Args:
        socketio: SocketIO instance
    """
    
    @socketio.on('connect')
    def handle_connect():
        """Handle WebSocket client connection."""
        logger.info(f'Client connected: {request.sid}')
        emit('connection_status', {'status': 'connected', 'message': 'Connected to DrRoboto WebSocket'})
        
        # Send current joint state if available
        last_joint_state = get_last_joint_state()
        if last_joint_state:
            joint_data = {
                'joint0': {
                    'position': last_joint_state.position[0] if len(last_joint_state.position) > 0 else 0.0,
                    'position_degrees': math.degrees(last_joint_state.position[0]) if len(last_joint_state.position) > 0 else 0.0,
                    'velocity': last_joint_state.velocity[0] if len(last_joint_state.velocity) > 0 else 0.0,
                    'effort': last_joint_state.effort[0] if len(last_joint_state.effort) > 0 else 0.0
                },
                'joint1': {
                    'position': last_joint_state.position[1] if len(last_joint_state.position) > 1 else 0.0,
                    'position_degrees': math.degrees(last_joint_state.position[1]) if len(last_joint_state.position) > 1 else 0.0,
                    'velocity': last_joint_state.velocity[1] if len(last_joint_state.velocity) > 1 else 0.0,
                    'effort': last_joint_state.effort[1] if len(last_joint_state.effort) > 1 else 0.0
                },
                'joint2': {
                    'position': last_joint_state.position[2] if len(last_joint_state.position) > 2 else 0.0,
                    'position_degrees': math.degrees(last_joint_state.position[2]) if len(last_joint_state.position) > 2 else 0.0,
                    'velocity': last_joint_state.velocity[2] if len(last_joint_state.velocity) > 2 else 0.0,
                    'effort': last_joint_state.effort[2] if len(last_joint_state.effort) > 2 else 0.0
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
        last_joint_state = get_last_joint_state()
        if last_joint_state:
            joint_data = {
                'joint0': {
                    'position': last_joint_state.position[0] if len(last_joint_state.position) > 0 else 0.0,
                    'position_degrees': math.degrees(last_joint_state.position[0]) if len(last_joint_state.position) > 0 else 0.0,
                    'velocity': last_joint_state.velocity[0] if len(last_joint_state.velocity) > 0 else 0.0,
                    'effort': last_joint_state.effort[0] if len(last_joint_state.effort) > 0 else 0.0
                },
                'joint1': {
                    'position': last_joint_state.position[1] if len(last_joint_state.position) > 1 else 0.0,
                    'position_degrees': math.degrees(last_joint_state.position[1]) if len(last_joint_state.position) > 1 else 0.0,
                    'velocity': last_joint_state.velocity[1] if len(last_joint_state.velocity) > 1 else 0.0,
                    'effort': last_joint_state.effort[1] if len(last_joint_state.effort) > 1 else 0.0
                },
                'joint2': {
                    'position': last_joint_state.position[2] if len(last_joint_state.position) > 2 else 0.0,
                    'position_degrees': math.degrees(last_joint_state.position[2]) if len(last_joint_state.position) > 2 else 0.0,
                    'velocity': last_joint_state.velocity[2] if len(last_joint_state.velocity) > 2 else 0.0,
                    'effort': last_joint_state.effort[2] if len(last_joint_state.effort) > 2 else 0.0
                },
                'timestamp': time.time()
            }
            emit('joint_state_update', joint_data)
        else:
            emit('joint_state_update', {'error': 'No joint state data available'})

    @socketio.on('set_joint')
    def handle_set_joint(data):
        """Handle individual joint control via Socket.IO."""
        ros_node = get_ros_node()
        if not ros_node:
            emit('error', {'message': 'ROS 2 not initialized'})
            return
        
        try:
            joint = data.get('joint')
            value = data.get('value')
            
            if joint is None or value is None:
                emit('error', {'message': 'Missing joint or value parameter'})
                return
            
            # Convert to degrees for logging
            degrees = value * 180 / math.pi
            
            logger.info(f"Socket.IO: Setting joint {joint} to {value:.3f} rad ({degrees:.1f}°)")
            
            # Get current joint states and update the specified joint
            last_joint_state = get_last_joint_state()
            if last_joint_state:
                joint0 = last_joint_state.position[0] if joint != 0 else value
                joint1 = last_joint_state.position[1] if joint != 1 else value
                joint2 = last_joint_state.position[2] if joint != 2 else value
            else:
                # Use home pose as fallback
                joint0, joint1, joint2 = HOME_POSE
                if joint == 0:
                    joint0 = value
                elif joint == 1:
                    joint1 = value
                elif joint == 2:
                    joint2 = value
            
            # Publish the updated joint state
            ros_node.publish_joint_state(joint0, joint1, joint2)
            
            emit('joint_updated', {
                'joint': joint,
                'value': value,
                'degrees': degrees,
                'success': True
            })
            
        except Exception as e:
            logger.error(f"Error setting joint via Socket.IO: {e}")
            emit('error', {'message': str(e)})


    @socketio.on('reset_joints')
    def handle_reset_joints():
        """Handle joint reset via Socket.IO."""
        ros_node = get_ros_node()
        if not ros_node:
            emit('error', {'message': 'ROS 2 not initialized'})
            return
        
        try:
            joint0, joint1, joint2 = HOME_POSE
            ros_node.publish_joint_state(joint0, joint1, joint2)
            
            emit('joints_reset', {
                'pose': HOME_POSE,
                'success': True
            })
            
        except Exception as e:
            logger.error(f"Error resetting joints via Socket.IO: {e}")
            emit('error', {'message': str(e)})

    @socketio.on('move_to_home')
    def handle_move_to_home():
        """Handle move to home via Socket.IO."""
        ros_node = get_ros_node()
        if not ros_node:
            emit('error', {'message': 'ROS 2 not initialized'})
            return
        
        try:
            joint0, joint1, joint2 = HOME_POSE
            ros_node.publish_joint_state(joint0, joint1, joint2)
            
            emit('moved_to_home', {
                'pose': HOME_POSE,
                'success': True
            })
            
        except Exception as e:
            logger.error(f"Error moving to home via Socket.IO: {e}")
            emit('error', {'message': str(e)})
