#!/usr/bin/env python3

import sys, select, termios, tty, math, threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty, Int32, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

MAX_LIN_VEL = 0.15
MAX_ANG_VEL = 4.5

MAX_J1 = 90
MIN_J1 = 0
MAX_J2 = 90
MIN_J2 = 0

MAX_TOOL = 45
MIN_TOOL = 0

STEP_LIN = 0.01
STEP_ANG = 0.5
STEP_ARM = 2

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

        # -- Subscribers --
        self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 1)
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)

        # -- Publishers --
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.joint_pub = self.create_publisher(Int32MultiArray, '/joint', 1)
        self.tool_pub = self.create_publisher(Int32, '/tool', 1)

        # -- Param --
        self.lin_vel = self.ang_vel = 0.0
        self.get_arm_data = False
        self.j1 = self.j2 = self.tool = 0

        self.settings = termios.tcgetattr(sys.stdin)

    # -- Subscriber callbacks --
    def cb_joint_states(self, msg):
        if not self.get_arm_data:
            self.j1 = int(msg.position[4] * 180 / math.pi)
            self.j2 = int(msg.position[5] * 180 / math.pi)
            self.tool = int(msg.position[6] * 180 / math.pi)
            print(f'[ ARM  ] init : j1={self.j1}, j2={self.j2}, tool={self.tool}')
            self.get_arm_data = True

    def cb_reset(self, msg):
        self.j1 = 0
        self.j2 = 0
        self.tool = 0
        self.lin_vel = 0.0
        self.ang_vel = 0.0

    # -- Functions --
    def fn_get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def fn_wheel_pub(self):
        print('[ WHEEL ] linear : %.2f, angular : %.2f' % (self.lin_vel, self.ang_vel))
        twist = Twist()
        twist.linear.x  = self.lin_vel
        twist.angular.z = self.ang_vel
        self.cmd_pub.publish(twist)

    def fn_joint_pub(self):
        print('[  ARM  ] j1 : %d, j2 : %d, tool : %d' % (self.j1, self.j2, self.tool))
        joint = Int32MultiArray()
        joint.data = [self.j1, self.j2]
        self.joint_pub.publish(joint)

        tool = Int32()
        tool.data = self.tool
        self.tool_pub.publish(tool)
    
    def fn_run(self):
        
        print(msg)
        status = 0
        try:
            while True:
                key = self.fn_get_key()
 
                if key == 'w':
                    self.lin_vel = constrain(self.lin_vel + STEP_LIN, MAX_LIN_VEL, -MAX_LIN_VEL)
                    self.fn_wheel_pub()
                    status += 1
                elif key == 'x':
                    self.lin_vel = constrain(self.lin_vel - STEP_LIN, MAX_LIN_VEL, -MAX_LIN_VEL)
                    self.fn_wheel_pub()
                    status += 1
                elif key == 'a':
                    self.ang_vel = constrain(self.ang_vel + STEP_ANG, MAX_ANG_VEL, -MAX_ANG_VEL)
                    self.fn_wheel_pub()
                    status += 1
                elif key == 'd':
                    self.ang_vel = constrain(self.ang_vel - STEP_ANG, MAX_ANG_VEL, -MAX_ANG_VEL)
                    self.fn_wheel_pub()
                    status += 1
                elif key == 'u':
                    self.j1 = constrain(self.j1 + STEP_ARM, MAX_J1, MIN_J1)
                    self.fn_arm_pub()
                    status += 1
                elif key == 'j':
                    self.j1 = constrain(self.j1 - STEP_ARM, MAX_J1, MIN_J1)
                    self.fn_arm_pub()
                    status += 1
                elif key == 'i':
                    self.j2 = constrain(self.j2 + STEP_ARM, MAX_J2, MIN_J2)
                    self.fn_arm_pub()
                    status += 1
                elif key == 'k':
                    self.j2 = constrain(self.j2 - STEP_ARM, MAX_J2, MIN_J2)
                    self.fn_arm_pub()
                    status += 1
                elif key == 'o':
                    self.tool = constrain(self.tool + STEP_ARM, MAX_TOOL, MIN_TOOL)
                    self.fn_arm_pub()
                    status += 1
                elif key == 'l':
                    self.tool = constrain(self.tool - STEP_ARM, MAX_TOOL, MIN_TOOL)
                    self.fn_arm_pub()
                    status += 1
                elif key == 'h':
                    self.j1 = self.j2 = self.tool = 0
                    self.fn_arm_pub()
                    status += 1
                elif key == 's':
                    self.lin_vel = 0.0
                    self.ang_vel = 0.0
                    self.fn_wheel_pub()
                    status += 1
                elif key == '\x03':  # CTRL-C
                    break
 
                if status == 20:
                    print(msg)
                    status = 0
 
        finally:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            self.fn_wheel_pub()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = AICAR_TELEOP()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.fn_run()

    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
