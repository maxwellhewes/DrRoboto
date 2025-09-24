#!/usr/bin/env python3
"""
Main entry point for the DrRoboto web application
This script can be run directly and handles the relative import issues
"""

import sys
import os

# Add the current directory to Python path so we can import the webapp package
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Now we can import and run the app
from app import app, socketio
from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG
from ros_controller import init_ros
from config import ROS_TOPIC_PREFIX
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

if __name__ == '__main__':
    logger.info("Starting DrRoboto Web Application...")
    logger.info(f"Environment variables: ROS_TOPIC_PREFIX='{ROS_TOPIC_PREFIX}'")
    
    # Initialize ROS 2
    logger.info("Attempting to initialize ROS 2...")
    if init_ros():
        # Set socketio reference for ROS controller to avoid circular imports
        from ros_controller import get_ros_node
        ros_node = get_ros_node()
        if ros_node:
            ros_node._socketio = socketio
        
        logger.info("Starting Flask-SocketIO app with ROS 2 support...")
        socketio.run(app, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG, allow_unsafe_werkzeug=True)
    else:
        logger.warning("Failed to initialize ROS 2. Starting Flask-SocketIO app without ROS 2 support...")
        socketio.run(app, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG, allow_unsafe_werkzeug=True)
