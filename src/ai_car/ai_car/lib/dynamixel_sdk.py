#!/usr/bin/env python3

import time

# 常用位址常數範例 (Protocol 1.0 / 2.0 通用概念)
ADDR_TORQUE_ENABLE = 24  # 註：Protocol 1.0 的 Torque 常常在位址 24，若是 Protocol 2.0 請依實際調整
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_MOVING_SPEED = 32

DXL_ON = 1
DXL_OFF = 0

class PortHandler:
    """模擬 Serial Port 連接"""
    def __init__(self, port_name):
        self.port_name = port_name
        self.is_open = False
        self.is_using = False  # 修正官方 SDK 檢查屬性報錯的問題

    def openPort(self):
        self.is_open = True
        self.is_using = True
        print(f"成功開啟通訊埠: {self.port_name}")
        return True

    def closePort(self):
        self.is_open = False
        self.is_using = False
        print("關閉通訊埠")

    def setBaudRate(self, baudrate):
        print(f"設定鮑率為: {baudrate}")
        return True


class PacketHandler:
    """模擬封包處理器與馬達內部記憶體 (Control Table)"""
    def __init__(self, protocol_version=1.0):
        self.protocol_version = protocol_version
        # 模擬馬達暫存器資料
        self.control_table = {
            ADDR_TORQUE_ENABLE: DXL_OFF,
            ADDR_MOVING_SPEED: 0,
            ADDR_GOAL_POSITION: 512,
            ADDR_PRESENT_POSITION: 512
        }

    def write1ByteTxRx(self, port, dxl_id, address, data):
        if not port.is_open:
            return -1, 1
        self.control_table[address] = data
        return 0, 0  # COMM_SUCCESS = 0

    def write2ByteTxRx(self, port, dxl_id, address, data):
        """新增的 2-Byte 寫入函式 (常用於 Protocol 1.0 的速度或位置控制)"""
        if not port.is_open:
            return -1, 1
        self.control_table[address] = data
        print(f"馬達 [ID: {dxl_id}] 寫入 2-Byte 到位址 {address}: {data}")
        return 0, 0  # COMM_SUCCESS = 0

    def write4ByteTxRx(self, port, dxl_id, address, data):
        if not port.is_open:
            return -1, 1
        self.control_table[address] = data
        print(f"馬達 [ID: {dxl_id}] 寫入 4-Byte 到位址 {address}: {data}")
        return 0, 0

    def read2ByteTxRx(self, port, dxl_id, address):
        if not port.is_open:
            return 0, -1, 1
        data = self.control_table.get(address, 0)
        return data, 0, 0

    def read4ByteTxRx(self, port, dxl_id, address):
        if not port.is_open:
            return 0, -1, 1
        data = self.control_table.get(address, 0)
        return data, 0, 0