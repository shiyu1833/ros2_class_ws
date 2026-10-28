#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LIDAR_SUB(Node):
    def __init__(self):
        super().__init__('lidar_follow')
        self.get_logger().info('lidar_follow START !!')

        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.scan_received = False
        self.scan_data = LaserScan()
        self.create_timer(0.3, self.cb_timer)

    def cb_scan(self, msg):
        self.scan_data = msg
        self.scan_received = True
        pass

    def cb_timer(self):
        if not self.scan_received:
            return
        
        detect_range = self.scan_data.ranges[175:186]
        right_range = self.scan_data.ranges[165:176]
        left_range = self.scan_data.ranges[185:196]

        detect_range = [x for x in detect_range if self.scan_data.range_min <= x <= self.scan_data.range_max]
        right_range = [x for x in right_range if self.scan_data.range_min <= x <= self.scan_data.range_max]
        left_range = [x for x in left_range if self.scan_data.range_min <= x <= self.scan_data.range_max]


        if len(detect_range) > 0: 
            detect_range = sum(detect_range) / len(detect_range)
        else:
            detect_range = 0.0

        if len(right_range) > 0: 
            right_range = sum(right_range) / len(right_range)
        else:
            right_range = 0.0

        if len(left_range) > 0: 
            left_range = sum(left_range) / len(left_range)
        else:
            left_range = 0.0

        print(f"D: {detect_range:.2f} m, R: {right_range:.2f} m, L: {left_range:.2f} m")

        cmd = Twist()

        if 0.4 <=detect_range <= 0.6:
            cmd.linear.x = 0.05
            if right_range < left_range and abs(right_range - left_range) > 0.2:
                cmd.linear.x = 0.0
                cmd.angular.z = -0.2
            elif right_range > left_range and abs(right_range - left_range) > 0.2:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.2
            else:
                cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


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