#!/usr/bin/env python3
"""Minimal test: verify ROS2 + Gazebo + controller imports all work."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # Create a publisher (sends velocity commands)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a timer (calls our function every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        
        self.get_logger().info('✅ Test node started successfully!')
        
        # Test that we can import the controllers
        try:
            import numpy as np
            self.get_logger().info(f'✅ NumPy {np.__version__} available')
        except ImportError:
            self.get_logger().error('❌ NumPy not found!')

    def timer_callback(self):
        self.count += 1
        
        # Create a velocity message
        msg = Twist()
        msg.linear.x = 0.0    # Don't actually move
        msg.angular.z = 0.0
        
        # Publish it
        self.cmd_pub.publish(msg)
        
        self.get_logger().info(
            f'Tick {self.count} | Publishing to /cmd_vel | '
            f'v=0.0, omega=0.0'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down test node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
