#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, Empty, Float32MultiArray, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from dynamixel_sdk import PortHandler
from ai_car.dxl_wheel import Wheel_Controller
from ai_car.dxl_arm import Arm_Controller

BAUD_RATE = 1000000
WHEEL_SEPERATION = 0.16
LIMIT_VEL = 200
VELOCITY2VALUE = 41.69988758
XM_TICK2RAD = 0.001533981

def constrain(vel):
    if vel < -LIMIT_VEL: return -LIMIT_VEL
    if vel > LIMIT_VEL: return LIMIT_VEL
    return vel

def calc_diff(current, last, total, threshold=2*XM_TICK2RAD):
    """
    計算 dxl 的差分與累計值
    current: 當前讀取值
    last: 上一次讀取值
    total: 累計角度
    threshold: 小於此閾值視為 0（抖動過濾）
    返回: diff, 更新後的 last, 更新後的 total
    """
    diff = (current - last) * XM_TICK2RAD
    if abs(diff) <= threshold:
        diff = 0.0
    total += diff
    last = current
    return diff, last, total

class AICAR_BASE(Node):
    def __init__(self):
        super().__init__('aicar_base')

        # --- init Dynamixel port ---
        self.dxl_port = PortHandler('/dev/U2D2')
        self.get_logger().info('Connect U2D2 ...')
        try:
            self.dxl_port.setBaudRate(BAUD_RATE)
            self.get_logger().info('[ OK ] AICAR start')
        except Exception as e:
            self.get_logger().error(f'{e}')
            raise SystemExit
        
        self.arm = Arm_Controller(self.dxl_port)
        self.wheel = Wheel_Controller(self.dxl_port)

        # --- Subscribers ---
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)
        self.create_subscription(Int8, '/torque', self.cb_torque, 1)
        self.create_subscription(Int32, '/tool', self.cb_tool, 1)
        self.create_subscription(Int32MultiArray, '/joint', self.cb_joint, 1)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 1)

        # --- Publishers ---
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.diff_rad_pub = self.create_publisher(Float32MultiArray, '/diff_rad', 1)

        # --- JointState ---
        self.joint_states = JointState()
        self.joint_states.name = [
            'wheel_left_joint',
            'wheel_right_joint',
            'wheel_rear_left_joint',
            'wheel_rear_right_joint',
            'arm_joint_1',
            'arm_joint_2',
            'tool_joint'
        ]

        # --- Param ---
        self.torque_data  = 0
        self.tool_data    = 0.0
        self.joint_data   = [0.0, 0.0]
        self.cmd_data     = [0.0, 0.0]
 
        self.reset_received  = False
        self.torque_received = False
        self.tool_received   = False
        self.joint_received  = False
        self.cmd_received    = False
 
        self.state_is_reset = False
        self.last_left = self.last_right = self.last_rear_left = self.last_rear_right = 0
        self.diff_left = self.diff_right = self.diff_rear_left = self.diff_rear_right = 0
        self.total_left = self.total_right = self.total_rear_left = self.total_rear_right = 0
 
        # --- main timer (30 Hz) ---
        self.create_timer( 1.0 / 30.0, self.cb_timer)

    # --- Subscriber callbacks ---
    def cb_reset(self, msg):
        self.get_logger().info(f'[ SET ] AICAR reset')
        self.total_left = self.total_right = self.last_rear_left = self.last_rear_right = 0
        self.state_is_reset = False
        self.reset_received = True

    def cb_torque(self, msg):
        self.get_logger().info(f'[ SET ] ARM Torque : {msg.data}')
        self.torque_data = msg.data
        self.torque_received = True

    def cb_tool(self, msg):
        self.get_logger().info(f'[ SET ] ARM Tool : {msg.data:.2f}')
        if not self.tool_received:
            self.tool_data = msg.data
            self.tool_received = True

    def cb_joint(self, msg):
        self.get_logger().info(f'[ SET ] ARM Joint : {msg.data[0]:.2f}, {msg.data[1]:.2f}')
        if not self.joint_received:
            self.joint_data = msg.data
            self.joint_received = True

    def cb_cmd(self, msg):
        if not self.cmd_received:
            lin_vel = msg.linear.x
            ang_vel = msg.angular.z

            left_vel = lin_vel - ang_vel * WHEEL_SEPERATION / 2
            right_vel = lin_vel + ang_vel * WHEEL_SEPERATION / 2

            left_vel = left_vel * VELOCITY2VALUE / 0.033
            right_vel = right_vel * VELOCITY2VALUE / 0.033

            left_vel = constrain(left_vel)
            right_vel = constrain(right_vel)

            self.get_logger().info(f'[ SET ] WHEEL : {int(left_vel)}, {int(right_vel)}')
            self.cmd_data = [left_vel, right_vel]
            self.cmd_received = True

    def cb_timer(self):
        self.fn_update_states()

        # Pub joint_states
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position = [
            self.total_left, self.total_right,
            self.total_rear_left, self.total_rear_right,
            self.arm_states[0], self.arm_states[1], self.arm_states[2]
        ]
        self.joint_states_pub.publish(self.joint_states)

        # Pub diff_rad
        diff_rad = Float32MultiArray()
        diff_rad.data = [float(self.diff_left), float(self.diff_right)]
        self.diff_rad_pub.publish(diff_rad)

        # 
        if self.tool_received:
            self.arm.set_tool(self.tool_data)
            self.tool_received = False

        if self.joint_received:
            self.arm.set_joint(self.joint_data)
            self.joint_received = False

        if self.cmd_received:
            self.wheel.set_wheel(self.cmd_data)
            self.cmd_received = False

        if self.reset_received:
            self.arm.reset()
            self.wheel.reset()

            self.create_timer(2.0, self.cb_flag)
            self.reset_received = False

        if self.torque_received:
            self.arm.set_torque(self.torque_data)
            self.torque_received = False

    def cb_flag(self):
        self.state_is_reset = False

    # --- Functions ---
    def fn_update_states(self):
        self.arm_states = self.arm.get_arm()
        self.wheel_states = self.wheel.get_wheel()

        if not self.state_is_reset:
            self.last_left = self.wheel_states[0]
            self.last_right = self.wheel_states[1]
            self.last_rear_left = self.wheel_states[2]
            self.last_rear_right = self.wheel_states[3]
            self.state_is_reset = True

        self.diff_left, self.last_left, self.total_left = calc_diff(self.wheel_states[0], 
                                                                    self.last_left, 
                                                                    self.total_left)

        self.diff_right, self.last_right, self.total_right = calc_diff(self.wheel_states[1], 
                                                                       self.last_right, 
                                                                       self.total_right)

        self.diff_rear_left, self.last_rear_left, self.total_rear_left = calc_diff(self.wheel_states[2], 
                                                                                   self.last_rear_left, 
                                                                                   self.total_rear_left)

        self.diff_rear_right, self.last_rear_right, self.total_rear_right = calc_diff(self.wheel_states[3], 
                                                                                      self.last_rear_right, 
                                                                                      self.total_rear_right)

    def fn_shutdown(self):
        print('Exiting ...')
        self.arm.reset()
        self.wheel.set_wheel([0, 0])
        time.sleep(3)
        self.arm.set_torque(0)
        self.wheel.set_torque(0)
        self.dxl_port.closePort()

def main():
    rclpy.init()
    node = AICAR_BASE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fn_shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
