#!/usr/bin/env python3

import smbus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu

I2C_BUS     = 1
I2C_ADDRESS = 0x68
IMU_RATE    = 200  # Hz

# MPU9250 registers
PWR_MGMT_1  = 0x6B
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47

# GYRO_CONFIG: ±250DPS → 0x00, ±500DPS → 0x08, ±1000DPS → 0x10, ±2000DPS → 0x18
GYRO_RANGE      = 0x00
GYRO_SENSITIVITY = 131.0   # LSB per deg/s at ±250DPS


class AICAR_IMU(Node):
    def __init__(self):
        super().__init__('aicar_imu')

        self.bus = smbus.SMBus(I2C_BUS)
        self.fn_init()

        # --- Subscribers ---
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)

        # --- Publishers ---
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        # --- Params ---
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'
        self.imu_msg.orientation_covariance[0]         = -1.0
        self.imu_msg.linear_acceleration_covariance[0] = -1.0

        self.create_timer(1.0 / IMU_RATE, self.cb_timer)

    # -- Subscriber callbacks --
    def cb_timer(self):
        gz = self.fn_read_gyro_z() * 0.017453293

        if abs(gz) >= 2.0:
            gz = 0.0

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.angular_velocity.z = gz
        self.imu_pub.publish(self.imu_msg)

    def cb_reset(self, msg):
        self.get_logger().info('[ SET ] IMU reset')
        self.fn_init()

    # -- Functions --
    def fn_init(self):
        self.bus.write_byte_data(I2C_ADDRESS, PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(I2C_ADDRESS, GYRO_CONFIG, GYRO_RANGE)
        self.get_logger().info('IMU init OK')

    def fn_read_gyro_z(self):
        high = self.bus.read_byte_data(I2C_ADDRESS, GYRO_ZOUT_H)
        low  = self.bus.read_byte_data(I2C_ADDRESS, GYRO_ZOUT_H + 1)
        val  = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val / GYRO_SENSITIVITY


def main():
    rclpy.init()
    node = AICAR_IMU()
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
