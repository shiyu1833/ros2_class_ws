#!/usr/bin/env python3

import math, rclpy, sys, select, termios, threading, tty

from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Int32MultiArray
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist


# ── 速度上限 ────────────────────────────────────────────────
MAX_LIN_VEL = 0.15
MAX_ANG_VEL = 4.5

# ── 手臂角度範圍 (度) ───────────────────────────────────────
MAX_J1 = 90;  MIN_J1 = 0
MAX_J2 = 90;  MIN_J2 = 0
MAX_TOOL = 90; MIN_TOOL = 0

# ── 每次按鍵的步進量 ────────────────────────────────────────
STEP_LIN = 0.01
STEP_ANG = 1.0
STEP_ARM = 1

# ── 按鍵配置 ───────────────────────────────────────────────
class KEYS():
    KEY_FRONT         = 'w'
    KEY_BACK          = 'x'
    KEY_LEFT          = 'a'
    KEY_RIGHT         = 'd'
    KEY_JOINT1_ADD    = 'u'
    KEY_JOINT1_SUB    = 'j'
    KEY_JOINT2_ADD    = 'i'
    KEY_JOINT2_SUB    = 'k'
    KEY_TOOL_ADD      = 'o'
    KEY_TOOL_SUB      = 'l'
    KEY_ARM_HOME_POSE = 'h'
    KEY_FORCE_STOP    = 's'
    KEY_CTRL_C        = '\x03'

msg = """
Control Your AICAR!
----------------------------------------------
Moving around:
            w(+)
   a(+)      s      d(-)
            x(-)
 
Arm control:
    Joint1 : u(+), j(-)
    Joint2 : i(+), k(-)
     Tool  : o(+), l(-)
 
h : Arm Home Pose
s : force stop
 
CTRL-C to quit
"""

def constrain(vel, max, min):
    if vel < min: return min
    if vel > max: return max
    return vel

class AICAR_TELEOP(Node):
    def __init__(self):
        super().__init__('aicar_teleop')

        # --- Subscribers ---
        # TODO: 建立以下兩個 Subscriber，綁定對應的 callback
        #   /joint_states  (JointState) → cb_joint_states
        #   /reset         (Empty)      → cb_reset
        self.subscription = self.create_subscription(JointState, "/joint_states", self.cb_joint_states, 1)
        self.subscription = self.create_subscription(Empty, "/reset", self.cb_reset, 1)
        self.subscription

        # --- Publishers ---
        # TODO: 建立以下三個 Publisher
        #   /cmd_vel  (Twist)           queue=1  → self.cmd_pub
        #   /joint      (Int32MultiArray) queue=1  → self.joint_pub
        #   /tool     (Int32)           queue=1  → self.tool_pub
        self.cmd_pub   = self.create_publisher(Twist, "/cmd_vel", 1)
        self.joint_pub = self.create_publisher(Int32MultiArray, "/joint", 1)
        self.tool_pub  = self.create_publisher(Int32, "/tool", 1)

        # --- 狀態變數 ---
        self.lin_vel = self.ang_vel = 0.0
        self.get_arm_data = False
        self.j1 = self.j2 = self.tool = 0
        self.settings = termios.tcgetattr(sys.stdin)
        self.status = 0

    # ── Subscriber Callbacks ────────────────────────────────

    def cb_joint_states(self, msg):
        """
        TODO: 只在 get_arm_data 為 False 時處理：
        1. 從 msg.position[4~6] 讀取弧度值，換算成整數角度（* 180 / pi）
           分別存入 self.j1, self.j2, self.tool
        2. 印出 '[ ARM ] init : j1=..., j2=..., tool=...'
        3. 設定 get_arm_data = True
        """
        if not self.get_arm_data:
            self.j1 = msg.position[4] * 180 / math.pi
            self.j2 = msg.position[5] * 180 / math.pi
            self.tool = msg.position[6] * 180 / math.pi
            self.get_logger().info(f"[ ARM ] init : j1={self.j1}, j2={self.j2}, tool={self.tool}")
            self.get_arm_data = True

    def cb_reset(self, msg):
        """
        TODO:
        1. 印出 '[ SET ] TELEOP reset'
        2. 將 j1, j2, tool, lin_vel, ang_vel 全部歸零。
        """
        self.get_logger().info(f"[ SET ] TELEOP reset")
        self.lin_vel = self.ang_vel = 0.0
        self.j1 = self.j2 = self.tool = 0

    # ── 功能函式 ────────────────────────────────────────────

    def fn_get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def fn_wheel_pub(self):
        """
        TODO:
        1. 印出 '[ WHEEL ] linear : %.2f, angular : %.2f'
        2. 建立 Twist 訊息，填入 lin_vel / ang_vel 並發布
        """
        self.get_logger().info(f"[ WHEEL ] linear : {self.lin_vel:.2f}, angular : {self.ang_vel:.2f}")
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.cmd_pub.publish(twist)

    def fn_arm_pub(self):
        """
        TODO:
        1. 印出 '[  ARM  ] j1 : %d, j2 : %d, tool : %d'
        2. 建立 Int32MultiArray，填入 [j1, j2] 並發布到 /joint
        3. 建立 Int32，填入 tool 並發布到 /tool
        """
        self.get_logger().info(f"[  ARM  ] j1 : {self.j1}, j2 : {self.j2}, tool : {self.tool}")
        joint = Int32MultiArray()
        joint.data = [int(self.j1), int(self.j2)]
        self.joint_pub.publish(joint)
        tool = Int32()
        tool.data = int(self.tool)
        self.tool_pub.publish(tool)

    def fn_run(self):
        """
        TODO: 主鍵盤控制迴圈
        1. 印出操作說明 msg
        2. 進入 while True 迴圈，每次呼叫 fn_get_key() 取得按鍵
        3. 根據按鍵對應的動作（參考 msg 說明）：
               w / x  → lin_vel ± STEP_LIN，呼叫 fn_wheel_pub()
               a / d  → ang_vel ± STEP_ANG，呼叫 fn_wheel_pub()
               u / j  → j1      ± STEP_ARM，呼叫 fn_arm_pub()
               i / k  → j2      ± STEP_ARM，呼叫 fn_arm_pub()
               o / l  → tool    ± STEP_ARM，呼叫 fn_arm_pub()
               h      → j1=j2=tool=0，呼叫 fn_arm_pub()
               s      → lin_vel=ang_vel=0.0，呼叫 fn_wheel_pub()
               CTRL-C ('\x03') → 跳出迴圈
           每個有效按鍵讓 status += 1；status 達到 20 時重新印出 msg 並歸零
        4. finally 區塊：停止車輪並還原終端機設定
        """
        print(msg)
        try:
            while True:
                match self.fn_get_key():
                    case KEYS.KEY_FRONT:
                        self.lin_vel = constrain(self.lin_vel + STEP_LIN, MAX_LIN_VEL, -MAX_LIN_VEL)
                        self.fn_wheel_pub()
                        self.status += 1
                    case KEYS.KEY_BACK:
                        self.lin_vel = constrain(self.lin_vel - STEP_LIN, MAX_LIN_VEL, -MAX_LIN_VEL)
                        self.fn_wheel_pub()
                        self.status += 1
                    case KEYS.KEY_LEFT:
                        self.ang_vel = constrain(self.ang_vel + STEP_ANG, MAX_ANG_VEL, -MAX_ANG_VEL)
                        self.fn_wheel_pub()
                        self.status += 1
                    case KEYS.KEY_RIGHT:
                        self.ang_vel = constrain(self.ang_vel - STEP_ANG, MAX_ANG_VEL, -MAX_ANG_VEL)
                        self.fn_wheel_pub()
                        self.status += 1
                    case KEYS.KEY_JOINT1_ADD:
                        self.j1 = constrain(self.j1 + STEP_ARM, MAX_J1, MIN_J1)
                        self.fn_arm_pub()
                        self.status += 1
                    case KEYS.KEY_JOINT1_SUB:
                        self.j1 = constrain(self.j1 - STEP_ARM, MAX_J1, MIN_J1)
                        self.fn_arm_pub()
                        self.status += 1
                    case KEYS.KEY_JOINT2_ADD:
                        self.j2 = constrain(self.j2 + STEP_ARM, MAX_J2, MIN_J2)
                        self.fn_arm_pub()
                        self.status += 1
                    case KEYS.KEY_JOINT2_SUB:
                        self.j2 = constrain(self.j2 - STEP_ARM, MAX_J2, MIN_J2)
                        self.fn_arm_pub()
                        self.status += 1
                    case KEYS.KEY_TOOL_ADD:
                        self.tool = constrain(self.tool + STEP_ARM, MAX_TOOL, MIN_TOOL)
                        self.fn_arm_pub()
                        self.status += 1
                    case KEYS.KEY_TOOL_SUB:
                        self.tool = constrain(self.tool - STEP_ARM, MAX_TOOL, MIN_TOOL)
                        self.fn_arm_pub()
                        self.status += 1
                    case KEYS.KEY_ARM_HOME_POSE:
                        self.j1 = self.j2 = self.tool = 0
                        self.fn_arm_pub()
                    case KEYS.KEY_FORCE_STOP:
                        self.lin_vel = self.ang_vel = 0.0
                        self.fn_wheel_pub()
                    case KEYS.KEY_CTRL_C:
                        break
                    
                if self.status == 20:
                    print(msg)
                    self.status = 0
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = AICAR_TELEOP()

    # 用 MultiThreadedExecutor 在背景執行 spin，讓主執行緒跑鍵盤迴圈
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.fn_run()
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
