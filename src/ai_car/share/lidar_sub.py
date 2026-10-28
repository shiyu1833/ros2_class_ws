#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class LIDAR_SUB(Node):
    def __init__(self):
        super().__init__('lidar_sub')
        self.get_logger().info('lidar_sub START !!')

        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)

        self.scan_received = False
        self.scan_data = LaserScan()
        self.create_timer(0.3, self.cb_timer)

    def cb_scan(self, msg):
        self.scan_data = msg
        self.scan_received = True
        pass

    def cb_timer(self):
        # index = (角度 - angle_min) / angle_increment

        if not self.scan_received:
            return

        index_count = len(self.scan_data.ranges)
        i0 = round((math.radians(0) - self.scan_data.angle_min) / self.scan_data.angle_increment)
        i90 = round((math.radians(90) - self.scan_data.angle_min) / self.scan_data.angle_increment)
        i180 = round((math.radians(180) - self.scan_data.angle_min) / self.scan_data.angle_increment)
        i270 = round((math.radians(-90) - self.scan_data.angle_min) / self.scan_data.angle_increment)

        d0 = self.scan_data.ranges[i0]
        d90 = self.scan_data.ranges[i90]
        d180 = self.scan_data.ranges[i180]
        d270 = self.scan_data.ranges[i270]

        if not self.scan_data.range_min <= d0 <= self.scan_data.range_max: d0 = float('nan')
        if not self.scan_data.range_min <= d90 <= self.scan_data.range_max: d90 = float('nan')
        if not self.scan_data.range_min <= d180 <= self.scan_data.range_max: d180 = float('nan')
        if not self.scan_data.range_min <= d270 <= self.scan_data.range_max: d270 = float('nan')

        # 平均值區塊
        index_range = 5
        d0_avg = self.scan_data.ranges[i0 - index_range:i0 + index_range + 1]
        d90_avg = self.scan_data.ranges[i90 - index_range:i90 + index_range + 1]
        d180_avg = self.scan_data.ranges[i180 - index_range:i180 + index_range + 1]
        d270_avg = self.scan_data.ranges[i270 - index_range:i270 + index_range + 1]

        d0_avg = [x for x in d0_avg if self.scan_data.range_min <= x <= self.scan_data.range_max]
        d90_avg = [x for x in d90_avg if self.scan_data.range_min <= x <= self.scan_data.range_max]
        d180_avg = [x for x in d180_avg if self.scan_data.range_min <= x <= self.scan_data.range_max]
        d270_avg = [x for x in d270_avg if self.scan_data.range_min <= x <= self.scan_data.range_max]

        if len(d0_avg) > 0:
            d0_avg = sum(d0_avg) / len(d0_avg)
        else:
            d0_avg = float('nan')
            
        if len(d90_avg) > 0:
            d90_avg = sum(d90_avg) / len(d90_avg)
        else:
            d90_avg = float('nan')

        if len(d180_avg) > 0:
            d180_avg = sum(d180_avg) / len(d180_avg)
        else:
            d180_avg = float('nan')

        if len(d270_avg) > 0:
            d270_avg = sum(d270_avg) / len(d270_avg)
        else:
            d270_avg = float('nan')

        print(f'lidar 筆數: {index_count}\n'
            f'[0度], index[{i0}], 單點距離: {d0:.2f} m, 平均距離: {d0_avg:.2f} m\n'
            f'[90度] index[{i90}], 單點距離: {d90:.2f} m, 平均距離: {d90_avg:.2f} m\n'
            f'[180度] index[{i180}], 單點距離: {d180:.2f} m, 平均距離: {d180_avg:.2f} m\n'
            f'[270度] index[{i270}], 單點距離: {d270:.2f} m, 平均距離: {d270_avg:.2f} m\n')


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