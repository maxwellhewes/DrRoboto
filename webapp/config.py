#!/usr/bin/env python3
"""
Configuration Module
Contains configuration constants and settings for the DrRoboto web application
"""

import os

# ROS Configuration
ROS_TOPIC_PREFIX = os.getenv('ROS_TOPIC_PREFIX', '')

# Robot Configuration
# Default home position (joint0, joint1, joint2) in radians
HOME_POSE = [0.0, 0.0, 0.0]

# Predefined poses (if any are used in the application)
POSES = {
    'home': [0.0, 0.0, 0.0],
    'pose1': [0.0, 0.5, -0.5],
    'pose2': [0.0, 1.0, -1.0]
}

# Flask Configuration
FLASK_HOST = '0.0.0.0'
FLASK_PORT = 5000
FLASK_DEBUG = True

# WebSocket Configuration
CORS_ALLOWED_ORIGINS = "*"
