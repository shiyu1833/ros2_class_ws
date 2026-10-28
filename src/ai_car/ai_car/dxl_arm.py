#!/usr/bin/env python3

import time

from dynamixel_sdk import PortHandler, PacketHandler

BAUD_RATE = 1000000
ARM_JOINT_1 = 11            # 812 ~ 512     0 - 90
ARM_JOINT_2 = 12            # 512 - 212     0 - 90
ARM_TOOL = 13               # 420 - 612     0 - 90
AX_TORUQE_ADDRESS = 24
AX_GOAL_POSITION = 30
AX_MOVING_SPEED = 32
AX_PRESENT_POSITION = 36
AX_TICK2RAD = 0.005113269   # 300 / 1024 / 180 * pi
AX_TICK2DEG = 0.29296875    # 300 / 1024
HOME_POS = [0, 0]
HOME_TOOL = 0

def to_int32(value):
    if value > 0x7FFFFFFF: return value - 0x100000000
    return value

def to_uint32(value):
    return int(value) & 0xFFFFFFFF

class Arm_Controller():
    def __init__(self, port):
        self.port = port
        self.packet = PacketHandler(1.0)
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_1, AX_MOVING_SPEED, 100)
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_2, AX_MOVING_SPEED, 100)
        self.packet.write2ByteTxRx(self.port, ARM_TOOL, AX_MOVING_SPEED, 100)
        time.sleep(1)
        self.reset()

    def reset(self):
        self.set_joint(HOME_POS)
        self.set_tool(HOME_TOOL)
        
    def set_torque(self, set_data):
        set_data = to_uint32(set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_1, AX_TORUQE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_JOINT_2, AX_TORUQE_ADDRESS, set_data)
        self.packet.write1ByteTxRx(self.port, ARM_TOOL, AX_TORUQE_ADDRESS, set_data)

    def set_joint(self, set_data):
        set_data = [to_uint32(x) for x in set_data]
        set_data[0] = 812 - int(set_data[0] / AX_TICK2DEG)
        set_data[1] = 212 + int(set_data[1] / AX_TICK2DEG)
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_1, AX_GOAL_POSITION, set_data[0])
        self.packet.write2ByteTxRx(self.port, ARM_JOINT_2, AX_GOAL_POSITION, set_data[1])

    def set_tool(self, set_data):
        set_data = to_uint32(set_data)
        set_data = 420 + int(set_data / AX_TICK2DEG)
        self.packet.write2ByteTxRx(self.port, ARM_TOOL, AX_GOAL_POSITION, set_data)

    def get_arm(self):
        get_data = [0, 0, 0]
        get_data[0] = self.packet.read2ByteTxRx(self.port, ARM_JOINT_1, AX_PRESENT_POSITION)[0]
        get_data[1] = self.packet.read2ByteTxRx(self.port, ARM_JOINT_2, AX_PRESENT_POSITION)[0]
        get_data[2] = self.packet.read2ByteTxRx(self.port, ARM_TOOL, AX_PRESENT_POSITION)[0]
        get_data[0] = (812 - get_data[0]) * AX_TICK2RAD
        get_data[1] = (get_data[1] - 212) * AX_TICK2RAD
        get_data[2] = (get_data[2] - 420) * AX_TICK2RAD
        return get_data

if __name__ == '__main__':
    print('dynamixel arm controller')
    try:
        pass

    except KeyboardInterrupt:
        pass
