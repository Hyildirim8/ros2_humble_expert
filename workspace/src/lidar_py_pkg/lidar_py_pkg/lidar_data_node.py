#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarDataNode(Node):
    def __init__(self):
        super().__init__('lidar_data_node')
        self._subscriber = self.create_subscription(LaserScan, '/lidar/out', self.listener_callback, 10)
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def listener_callback(self, msg):
        min_lidar_data = min(msg.ranges)
        self.get_logger().info(f'Received LIDAR data with {min_lidar_data} as minimum range.')
        
        velocity_msg = Twist()
        if min_lidar_data < 2.0:
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.5
            self.get_logger().info('Obstacle detected! Stopping and turning.')
        else:
            velocity_msg.linear.x = 0.5
            velocity_msg.angular.z = 0.0
            self.get_logger().info('Path is clear. Moving forward.')
        
        self._publisher.publish(velocity_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = LidarDataNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()