#!/usr/bin/env python3
"""
Simple Joint State Publisher for Two Link Arm
Publishes joint states to visualize different poses
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
import math
import time

class SimpleJointController(Node):
    def __init__(self):
        super().__init__('simple_joint_controller')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber for pose commands
        self.pose_subscriber = self.create_subscription(
            String,
            '/arm_pose_command',
            self.pose_command_callback,
            10
        )
        
        # Publisher for status messages
        self.status_publisher = self.create_publisher(
            String,
            '/arm_status',
            10
        )
        
        # Define the three poses (joint1, joint2) in radians
        self.poses = {
            'home': [0.0, 0.0],           # Straight up
            'pose1': [math.pi/4, -math.pi/4],  # 45 degrees, -45 degrees
            'pose2': [-math.pi/4, math.pi/3],  # -45 degrees, 60 degrees
            'pose3': [math.pi/2, -math.pi/6]   # 90 degrees, -30 degrees
        }
        
        self.current_pose = 'home'
        self.get_logger().info('Simple Joint Controller initialized')
        self.get_logger().info(f'Available poses: {list(self.poses.keys())}')
        
        # Timer for automatic pose cycling (every 3 seconds)
        self.create_timer(3.0, self.cycle_poses)
        self.pose_index = 0
        
        # Start with home pose
        self.move_to_pose('home')

    def pose_command_callback(self, msg):
        """Handle incoming pose commands"""
        pose_name = msg.data.lower().strip()
        if pose_name in self.poses:
            self.move_to_pose(pose_name)
        else:
            self.get_logger().warn(f'Unknown pose: {pose_name}')
            self.get_logger().info(f'Available poses: {list(self.poses.keys())}')

    def cycle_poses(self):
        """Automatically cycle through poses for demonstration"""
        pose_names = list(self.poses.keys())
        pose_name = pose_names[self.pose_index]
        self.move_to_pose(pose_name)
        self.pose_index = (self.pose_index + 1) % len(pose_names)

    def move_to_pose(self, pose_name):
        """Move arm to specified pose"""
        if pose_name not in self.poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return
        
        joint_positions = self.poses[pose_name]
        self.current_joint_positions = joint_positions
        
        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2']
        joint_state.position = joint_positions
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        
        self.joint_pub.publish(joint_state)
        
        # Update current pose
        self.current_pose = pose_name
        
        # Log and publish status
        self.get_logger().info(f'Moved to pose: {pose_name} {joint_positions}')
        status_msg = String()
        status_msg.data = f'Moved to {pose_name}'
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = SimpleJointController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()