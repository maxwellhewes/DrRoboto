#!/usr/bin/env python3
"""
DrRoboto Web Control Interface
Flask web application for controlling the robotic arm simulation
Uses direct ROS 2 network communication instead of Docker exec
"""

import logging

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO

try:
    # Try relative imports first (when used as a package)
    from .config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG, CORS_ALLOWED_ORIGINS
    from .ros_controller import init_ros
    from .web_routes import register_routes
    from .websocket_handlers import register_websocket_handlers
except ImportError:
    # Fall back to absolute imports (when run directly)
    from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG, CORS_ALLOWED_ORIGINS
    from ros_controller import init_ros
    from web_routes import register_routes
    from websocket_handlers import register_websocket_handlers

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins=CORS_ALLOWED_ORIGINS)

# Enable Flask debug logging
app.logger.setLevel(logging.DEBUG)

# Register routes and WebSocket handlers
register_routes(app)
register_websocket_handlers(socketio)

# Set socketio reference for ROS controller to avoid circular imports
try:
    from .ros_controller import get_ros_node
except ImportError:
    from ros_controller import get_ros_node

def set_socketio_reference():
    """Set the socketio reference in the ROS controller after initialization."""
    ros_node = get_ros_node()
    if ros_node:
        ros_node._socketio = socketio


if __name__ == '__main__':
    logger.info("Starting DrRoboto Web Application...")
    try:
        from .config import ROS_TOPIC_PREFIX
    except ImportError:
        from config import ROS_TOPIC_PREFIX
    logger.info(f"Environment variables: ROS_TOPIC_PREFIX='{ROS_TOPIC_PREFIX}'")
    
    # Initialize ROS 2
    logger.info("Attempting to initialize ROS 2...")
    if init_ros():
        logger.info("Setting socketio reference for ROS controller...")
        set_socketio_reference()
        logger.info("Starting Flask-SocketIO app with ROS 2 support...")
        socketio.run(app, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG, allow_unsafe_werkzeug=True)
    else:
        logger.warning("Failed to initialize ROS 2. Starting Flask-SocketIO app without ROS 2 support...")
        socketio.run(app, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG, allow_unsafe_werkzeug=True)
