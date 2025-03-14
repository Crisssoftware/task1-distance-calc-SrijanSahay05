#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DistanceMonitor(Node):
    def __init__(self):
        super().__init__('distance_monitor')
        
        # Create subscription to the distance topic
        self.subscription = self.create_subscription(
            Float32,
            '/turtle1/distance_from_origin',
            self.distance_callback,
            10)
        
        self.get_logger().info('Distance monitor initialized. Waiting for messages...')

    def distance_callback(self, msg):
        # Extract distance value from message
        distance = msg.data
        
        # Print the distance with formatting
        self.get_logger().info(f'Distance from origin: {distance:.2f} units')

def main(args=None):
    rclpy.init(args=args)
    
    monitor = DistanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
