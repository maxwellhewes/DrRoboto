#!/usr/bin/env python3
"""
DrRoboto Web Application Package
Flask web application for controlling the robotic arm simulation
"""

from .app import app, socketio

__all__ = ['app', 'socketio']
