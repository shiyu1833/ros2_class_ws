#!/usr/bin/env python3

from math import sin, cos

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from std_msgs.msg import Empty, Float32MultiArray

import tf2_ros

WHEEL_RADIUS    = 0.033

class AICAR_ODOM(Node):
    def __init__(self):
        super().__init__('aicar_odom')

        # --- Waiting imu message ---
        self.get_logger().info('Waiting for /imu ...')
        self.imu_is_reset = False
        self.imu_ang      = 0.0

        # --- Params ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # --- Subscribers ---
        self.create_subscription(Imu, '/imu', self.cb_imu, 10)
        self.create_subscription(Float32MultiArray, '/diff_rad', self.cb_calc_odom, 10)
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # --- TF broadcaster ---
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Params ---
        self.odom             = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id  = 'base_footprint'

    # --- Subscriber Callbacks ---

    def cb_reset(self, msg):
        self.get_logger().info('[ SET ] Odom reset')
        self.imu_is_reset = False
        self.x = self.y = self.theta = 0.0
        self.imu_ang = 0.0

    def cb_imu(self, msg):
        self.imu_ang = msg.angular_velocity.z
        self.imu_is_reset = True

    def cb_calc_odom(self, msg):
        if not self.imu_is_reset:
            return

        if msg.data[0] == 0.0 and msg.data[1] == 0.0:
            ang = 0.0
        else:
            ang = self.imu_ang

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        l_distance = msg.data[0] * WHEEL_RADIUS
        r_distance = msg.data[1] * WHEEL_RADIUS

        lin = (l_distance + r_distance) / (2.0 * dt)

        delta_x     = lin * cos(self.theta) * dt
        delta_y     = lin * sin(self.theta) * dt
        delta_theta = ang * dt

        self.x     += delta_x
        self.y     += delta_y
        self.theta += delta_theta

        # --- Publish Odometry ---
        self.odom.header.stamp = current_time.to_msg()

        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0

        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = sin(self.theta / 2)
        self.odom.pose.pose.orientation.w = cos(self.theta / 2)

        self.odom.twist.twist.linear.x  = lin
        self.odom.twist.twist.angular.z = ang

        self.odom_pub.publish(self.odom)

        # --- Broadcast TF: odom → base_footprint ---
        t = TransformStamped()
        t.header.stamp    = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(self.theta / 2)
        t.transform.rotation.w = cos(self.theta / 2)

        self.broadcaster.sendTransform(t)

        self.last_time = current_time

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
