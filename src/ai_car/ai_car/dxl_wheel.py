#!/usr/bin/env python3

import time

from dynamixel_sdk import PortHandler, PacketHandler

BAUD_RATE = 1000000
LEFT_ID = 1
RIGHT_ID = 2
REAR_LEFT_ID = 3
REAR_RIGHT_ID = 4
XM_OPERATING_MODE_ADDRESS = 11
XM_TORQUE_ADDRESS = 64
XM_GOAL_VELOCITY_ADDRESS = 104
XM_PRESENT_POSITION_ADDRESS = 132
XM_TICK2RAD = 0.001533981

def to_int32(value):
    if value > 0x7FFFFFFF: return value - 0x100000000
    return value

def to_uint32(value):
    return int(value) & 0xFFFFFFFF

class Wheel_Controller():
    def __init__(self, port):
        self.port = port
        self.packet = PacketHandler(1.0)
        self.port.setBaudRate(BAUD_RATE)
        self.reset()

    def reset(self):
        self.set_torque(0)
        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_OPERATING_MODE_ADDRESS, 3)
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_OPERATING_MODE_ADDRESS, 3)
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_OPERATING_MODE_ADDRESS, 3)
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_OPERATING_MODE_ADDRESS, 3)
        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_OPERATING_MODE_ADDRESS, 1)
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_OPERATING_MODE_ADDRESS, 1)
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_OPERATING_MODE_ADDRESS, 1)
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_OPERATING_MODE_ADDRESS, 1)
        return self.set_torque(1)

    def set_torque(self, set_data):
        set_data = to_uint32(set_data)
        self.packet.write1ByteTxRx(self.port, LEFT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, RIGHT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, REAR_LEFT_ID, XM_TORQUE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, REAR_RIGHT_ID, XM_TORQUE_ADDRESS, set_data)
        return set_data

    def set_wheel(self, set_data):
        set_data = [to_uint32(x) for x in set_data]
        self.packet.write4ByteTxRx(self.port, LEFT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[0])
        self.packet.write4ByteTxRx(self.port, RIGHT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[1])
        self.packet.write4ByteTxRx(self.port, REAR_LEFT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[0])
        self.packet.write4ByteTxRx(self.port, REAR_RIGHT_ID, XM_GOAL_VELOCITY_ADDRESS, set_data[1])

    def get_wheel(self):
        get_data = [0,0,0,0]
        get_data[0] = self.packet.read4ByteTxRx(self.port, LEFT_ID, XM_PRESENT_POSITION_ADDRESS)[0]
        get_data[1] = self.packet.read4ByteTxRx(self.port, RIGHT_ID, XM_PRESENT_POSITION_ADDRESS)[0]
        get_data[2] = self.packet.read4ByteTxRx(self.port, REAR_LEFT_ID, XM_PRESENT_POSITION_ADDRESS)[0]
        get_data[3] = self.packet.read4ByteTxRx(self.port, REAR_RIGHT_ID, XM_PRESENT_POSITION_ADDRESS)[0]
        get_data = [to_int32(x) for x in get_data]
        return get_data
    
if __name__ == '__main__':
    print('dynamixel wheel controller')
    try:
        pass

    except KeyboardInterrupt:
        pass