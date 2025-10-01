#!/usr/bin/env python3
"""
Example script to test the matlab_slave node by publishing pose messages.
This simulates what MATLAB would send to the robot.

Usage:
    python3 test_matlab_slave.py

The script will publish a series of test poses to the 'matlab_slave' topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import time
import math

class MatlabSlaveTestPublisher(Node):
    def __init__(self):
        super().__init__('matlab_slave_test_publisher')
        
        # Create publisher for matlab_slave topic
        self.publisher = self.create_publisher(
            PoseStamped, 
            'matlab_slave', 
            10
        )
        
        # Timer to publish test poses periodically
        self.timer = self.create_timer(5.0, self.publish_test_pose)
        
        # Test poses counter
        self.pose_counter = 0
        
        # Define test poses (position + orientation)
        self.test_poses = [
            # Pose 1: Home position
            {
                'position': [0.3, 0.0, 0.5],
                'orientation': [0.0, 0.0, 0.0, 1.0],  # No rotation
                'description': 'Home position'
            },
            # Pose 2: Move to the right
            {
                'position': [0.3, -0.2, 0.5],
                'orientation': [0.0, 0.0, 0.7071, 0.7071],  # 90° rotation around Z
                'description': 'Right side position'
            },
            # Pose 3: Move to the left
            {
                'position': [0.3, 0.2, 0.5],
                'orientation': [0.0, 0.0, -0.7071, 0.7071],  # -90° rotation around Z
                'description': 'Left side position'
            },
            # Pose 4: Move forward and down
            {
                'position': [0.5, 0.0, 0.3],
                'orientation': [0.0, 0.3827, 0.0, 0.9239],  # 45° pitch down
                'description': 'Forward and down position'
            },
            # Pose 5: Return to center, higher up
            {
                'position': [0.3, 0.0, 0.6],
                'orientation': [0.0, 0.0, 0.0, 1.0],  # No rotation
                'description': 'Center high position'
            }
        ]
        
        self.get_logger().info('Matlab Slave Test Publisher initialized')
        self.get_logger().info(f'Will publish {len(self.test_poses)} test poses')
        self.get_logger().info('Publishing first pose in 5 seconds...')
    
    def publish_test_pose(self):
        if self.pose_counter >= len(self.test_poses):
            self.get_logger().info('All test poses published. Restarting sequence...')
            self.pose_counter = 0
            return
        
        # Get current test pose
        current_pose_data = self.test_poses[self.pose_counter]
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'  # Robot base frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose_msg.pose.position.x = current_pose_data['position'][0]
        pose_msg.pose.position.y = current_pose_data['position'][1]
        pose_msg.pose.position.z = current_pose_data['position'][2]
        
        # Set orientation (quaternion)
        pose_msg.pose.orientation.x = current_pose_data['orientation'][0]
        pose_msg.pose.orientation.y = current_pose_data['orientation'][1]
        pose_msg.pose.orientation.z = current_pose_data['orientation'][2]
        pose_msg.pose.orientation.w = current_pose_data['orientation'][3]
        
        # Publish the pose
        self.publisher.publish(pose_msg)
        
        # Log the published pose
        self.get_logger().info(
            f'Published pose {self.pose_counter + 1}/{len(self.test_poses)}: '
            f'{current_pose_data["description"]} - '
            f'Position: ({pose_msg.pose.position.x:.3f}, '
            f'{pose_msg.pose.position.y:.3f}, {pose_msg.pose.position.z:.3f})'
        )
        
        self.pose_counter += 1
    
    def publish_custom_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        Publish a custom pose with specified position and orientation.
        
        Args:
            x, y, z: Position coordinates (meters)
            qx, qy, qz, qw: Quaternion orientation (default: no rotation)
        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        self.publisher.publish(pose_msg)
        
        self.get_logger().info(
            f'Published custom pose: Position({x:.3f}, {y:.3f}, {z:.3f}), '
            f'Orientation({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = MatlabSlaveTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test publisher shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()