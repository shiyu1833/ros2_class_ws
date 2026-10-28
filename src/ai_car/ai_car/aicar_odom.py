#!/usr/bin/env python3

import rclpy, tf2_ros

from math import sin, cos
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

# ── 常數定義 ────────────────────────────────────────────────
WHEEL_RADIUS     = 0.033   # 輪子半徑 (m)
WHEEL_SEPARATION = 0.16    # 左右輪距 (m)

class AICAR_ODOM(Node):
    def __init__(self):
        super().__init__('aicar_odom')

        # --- 等待 IMU 訊息 ---
        self.get_logger().info('Waiting for /imu ...')
        self.imu_is_reset = False
        self.imu_ang      = 0.0

        # --- 里程計狀態變數 ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # --- Subscribers ---
        # TODO: 建立以下三個 Subscriber，綁定對應的 callback
        #   /imu       (Imu)                → cb_imu,       queue=10
        #   /diff_rad  (Float32MultiArray)  → cb_calc_odom, queue=10
        #   /reset     (Empty)              → cb_reset,      queue=1
        self.subscription = self.create_subscription(Imu, "/imu", self.cb_imu, 10)
        self.subscription = self.create_subscription(Float32MultiArray, "/diff_rad", self.cb_calc_odom, 10)
        self.subscription = self.create_subscription(Empty, "/reset", self.cb_reset, 1)
        self.subscription

        # --- Publishers ---
        # TODO: 建立以下一個 Publisher
        #   /odom  (Odometry)  queue=10
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # --- TF Broadcaster ---
        # TODO: 建立 tf2_ros.TransformBroadcaster 實例，指定給 self.broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Odometry 初始化 ---
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id  = 'base_footprint'

    # ── Subscriber Callbacks ────────────────────────────────

    def cb_reset(self, msg):
        """
        TODO:
        1. 印出 '[ SET ] Odom reset'
        2. 將 imu_is_reset 設為 False
        3. 將 x, y, theta 歸零
        4. 將 imu_ang 歸零
        """
        self.get_logger().info(f"[ SET ] Odom reset")
        self.imu_is_reset = False
        self.x = self.y = self.theta
        self.imu_ang = 0

    def cb_imu(self, msg):
        """
        TODO:
        1. 從 msg.angular_velocity.z 讀取角速度，儲存到 self.imu_ang
        2. 將 imu_is_reset 設為 True（表示 IMU 已就緒）
        """
        self.imu_ang = msg.angular_velocity.z
        self.imu_is_reset = True

    def cb_calc_odom(self, msg):
        """
        TODO:
        1. 若 imu_is_reset 為 False，直接 return（IMU 尚未就緒）

        2. 判斷這個時間是否有轉動：
           若左右輪的差分值都是 0，代表車子靜止不轉，角速度視為 0；
           否則使用 IMU 回報的角速度

        3. 取得當前時間，並計算距離上一次呼叫過了多久（dt，單位秒）；
           若時間差不合理（小於等於 0），直接離開

        4. 將左右輪的弧度差分乘上輪子半徑，換算成各自的實際行進距離

        5. 取左右輪行進距離的平均，除以 dt，得到車體的線速度

        6. 根據線速度、當前朝向角與 dt，計算這個時間步在 x、y 方向的位移，
           以及朝向角的變化量，並累加到目前的位姿上

        7. 將更新後的位姿與速度填入 Odometry 訊息並發布：
           位置填 x、y（z 為 0），
           朝向用四元數表示（僅繞 Z 軸旋轉），
           線速度與角速度也一併填入

        8. 廣播 TF 座標變換（odom → base_footprint）：
           平移部分填入目前的 x、y（z 為 0），
           旋轉部分與 Odometry 的四元數相同

        9. 將當前時間記錄為 last_time，供下次計算 dt 使用
        """
        if not self.imu_is_reset: return
        
        if msg.data[0] == 0 and msg.data[1] == 0: ang_vel = 0.0
        else: ang_vel = self.imu_ang

        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9

        if dt <= 0: return

        left_dist = msg.data[0] * WHEEL_RADIUS
        right_dist = msg.data[1] * WHEEL_RADIUS
        lin_vel = ((left_dist + right_dist) / 2) / dt
        # ang_vel = ((right_dist - left_dist) / WHEEL_SEPARATION ) / dt

        delta_x = lin_vel * cos(self.theta) *dt
        delta_y = lin_vel * sin(self.theta) *dt
        delta_theta = ang_vel * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        self.odom.header.stamp = curr_time.to_msg()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = sin(self.theta / 2)
        self.odom.pose.pose.orientation.w = cos(self.theta / 2)
        self.odom.twist.twist.linear.x = lin_vel
        self.odom.twist.twist.angular.z = ang_vel
        self.odom_pub.publish(self.odom)

        t = TransformStamped()
        t.header.stamp    = curr_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)

        self.broadcaster.sendTransform(t)
        self.last_time = curr_time

def main():
    rclpy.init()
    node = AICAR_ODOM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
