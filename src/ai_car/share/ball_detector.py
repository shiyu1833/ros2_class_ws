#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray, Int32
from sensor_msgs.msg import CompressedImage

# --- Blob Detection Params ---
MIN_RADIUS = 20
MAX_RADIUS = 0

# --- Arm Preset Positions ---
JOINT_READY = [0, 0]
JOINT_GRAB  = [60, 90]
TOOL_OPEN   = 90
TOOL_CLOSE  = 45


class BALL_DETECTOR(Node):
    def __init__(self):
        super().__init__('ball_detector')

        # --- CvBridge ---
        self.cvBridge = CvBridge()

        # --- Subscribers ---
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.cb_image, 10)

        # --- Publishers ---
        self.joint_pub = self.create_publisher(Int32MultiArray, '/joint', 1)
        self.tool_pub  = self.create_publisher(Int32,           '/tool',  1)
        self.image_pub = self.create_publisher(CompressedImage, '/image_detected/compressed', 10)

        # --- Param ---
        self.cv_bridge = CvBridge()
        self.image_data = None
        self.image_received = False

        self.ball_cx     = None
        self.ball_cy     = None
        self.ball_radius = None

        # --- State Machine ---
        self.state = 'STANDBY'
        self.joint_cmd = Int32MultiArray()
        self.tool_cmd  = Int32()

        # --- main timer (30 Hz) ---
        self.create_timer(1/30.0, self.cbTimer)

        self.get_logger().info('[ OK ] BALL_DETECTOR start')

    # --- Subscriber callbacks ---
    def cb_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image_data = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_received = True

    # --- Timer ---
    def cbTimer(self):
        if self.image_received:
            self.fnDetectBall()

        if self.ball_cx is not None and self.ball_cy is not None and self.ball_radius is not None:
            if self.state == 'STANDBY':
                self.get_logger().info('[ STATE ] STANDBY')
                self.joint_cmd.data = JOINT_READY
                self.tool_cmd.data  = TOOL_OPEN
                self.joint_pub.publish(self.joint_cmd)
                self.tool_pub.publish(self.tool_cmd)
                self.state = 'SCAN'

            elif self.state == 'SCAN' and  190 <= self.ball_cx <= 200 and 110 <= self.ball_cy <= 150 and self.ball_radius >= 60:
                self.get_logger().info('[ STATE ] SCAN')
                self.joint_cmd.data = JOINT_GRAB
                self.counter = 0
                self.joint_pub.publish(self.joint_cmd)
                self.state = 'GRAB'

        elif self.state == 'GRAB':
            self.get_logger().info('[ STATE ] GRAB')
            self.counter += 1
            if self.counter >= 60:
                self.tool_cmd.data  = TOOL_CLOSE
                self.tool_pub.publish(self.tool_cmd)
                self.counter = 0
                self.state = 'WAIT'

        elif self.state == 'WAIT':
            self.get_logger().info('[ STATE ] WAIT')
            self.counter += 1
            if self.counter >= 60:
                self.joint_cmd.data = JOINT_READY
                self.joint_pub.publish(self.joint_cmd)
                self.state = 'STANDBY'


    # --- Functions ---
    def fnDetectBall(self):
        image = np.copy(self.image_data)
        gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray  = cv2.GaussianBlur(gray, (5, 5), 0)

        h, w  = image.shape[:2]
        cv2.line(image, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        cv2.line(image, (0, h // 2), (w, h // 2), (255, 255, 0), 1)

        if self.ball_cx is not None and self.ball_cy is not None:
            cv2.circle(image, (self.ball_cx, self.ball_cy), self.ball_radius, (255, 255, 0), 2)
            cv2.drawMarker(image, (self.ball_cx, self.ball_cy), (255, 255, 0), cv2.MARKER_CROSS, 10, 2)
            cv2.putText(image,f'({self.ball_cx}, {self.ball_cy}, {self.ball_radius})',(10,60), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 1)

        self.image_pub.publish(self.cvBridge.cv2_to_compressed_imgmsg(image, 'jpg'))

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT_ALT,
            dp        = 1.5,
            minDist   = 200,    
            param1    = 100,    
            param2    = 0.9,
            minRadius = MIN_RADIUS,
            maxRadius = MAX_RADIUS
        )

        if circles is None:
            self.ball_cx = self.ball_cy = self.ball_radius = None
            return

        circles = np.uint16(np.around(circles))
        cx, cy, r = circles[0][0]

        self.ball_cx     = int(cx)
        self.ball_cy     = int(cy)
        self.ball_radius = int(r)

        # self.get_logger().info(f'[ BALL ] cx={self.ball_cx}, cy={self.ball_cy}, r={self.ball_radius}')

def main():
    rclpy.init()
    node = BALL_DETECTOR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()