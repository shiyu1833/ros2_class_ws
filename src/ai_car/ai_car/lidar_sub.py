#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class LIDAR_SUB(Node):
    def __init__(self):
        super().__init__('lidar_sub')
        self.get_logger().info('lidar_sub START !!')

        # TODO 建立以下一個 Subscriber，綁定對應的 callback
        #   /scan   (LaserScan)   → cb_scan,   queue=10

        self.scan_received = False
        self.scan_data = LaserScan()
        self.create_timer(0.3, self.cb_timer)

    def cb_scan(self, msg):
        # TODO 在此 callback 中儲存接收到的 LaserScan 訊息，並將 scan_received 標記為 True
        #   self.scan_data = ???
        #   self.scan_received = ???
        pass

    def cb_timer(self):
        if not self.scan_received:
            return

        # =====================================================================
        # 【練習一】按角度讀取Index索引值及光達距離
        #  顯示 出角度 - 索引 - 距離 對應數據，並將 inf 數值顯示 nan
        # =====================================================================

        index_count = len(self.scan_data.ranges)

        # TODO 
        # 1：計算各方向的 index，i0、i90、i180、i270 分別對應 0、90、180、270 度
        # 2：取出各方向的單點距離 d0、d90、d180、d270
        # 3：無效值處理
        #       超出 range_min ~ range_max 的數值設為 float('nan')
        #       RPLIDAR A1M8：range_min=0.15m, range_max=6.0m
        # 4：印出結果
        #   範例輸出格式：
        #     lidar 筆數: 360
        #     [0度]   index[180], 單點距離: 0.85 m
        #     [90度]  index[270], 單點距離: 1.20 m
        #     [180度] index[0],   單點距離: nan m
        #     [270度] index[90],  單點距離: 0.42 m


        

        # =====================================================================
        # 【練習二】取扇形數據平均讓數值更穩定
        # =====================================================================

        index_range = 5  # 前後各取幾點

        # TODO 
        # 1：使用切片取出各方向的 Index 扇形範圍，d0_avg、d90_avg、d180_avg、d270_avg
        # 2：過濾各陣列中的無效值
        # 3：計算平均值
        #       若過濾後陣列為空，設為 float('nan')
        # 4：印出結果
        #   範例輸出格式：
        #     lidar 筆數: 360
        #     [0度]   index[180], 單點距離: 0.85 m, 平均距離: 0.83 m
        #     [90度]  index[270], 單點距離: 1.20 m, 平均距離: 1.18 m
        #     [180度] index[0],   單點距離: nan m,  平均距離: nan m
        #     [270度] index[90],  單點距離: 0.42 m, 平均距離: 0.44 m
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