#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class LIDAR_SUB(Node):
    def __init__(self):
        super().__init__('lidar_follow')
        self.get_logger().info('lidar_follow START !!')

    def cb_scan(self, msg):
        pass

    def cb_timer(self):
        pass

def main():
    rclpy.init()
    node = LIDAR_SUB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        
if __name__ == '__main__':
    main()