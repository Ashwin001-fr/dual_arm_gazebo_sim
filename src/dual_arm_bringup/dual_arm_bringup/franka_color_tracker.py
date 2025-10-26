#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge


class SimpleFrankaTracker(Node):
    def __init__(self):
        super().__init__('simple_franka_tracker')
        self.bridge = CvBridge()

        # Initial joint angles (Franka 7-DOF)
        self.q_init = np.array([2.89, -0.785, 0.0, -2.356, 0.0, 1.571, -0.5], dtype=float)
        self.q = self.q_init.copy()

        self.joint_limits = [(-2.89, 2.89), (-1.76, 1.76), (-2.89, 2.89),
                             (-3.07, -0.07), (-2.89, 2.89), (-0.01, 3.75), (-2.89, 2.89)]

        # SMOOTHER Control gains
        self.Kp_x = 0.0015 
        self.Kp_y = 0.0015  
        self.Kp_z = 0.001
        self.max_step = 0.03  

        self.joint4_scale = 1

        # Larger deadzone for more stability
        self.deadzone_px = 40  

        # STRONGER smoothing for smoother motion
        self.alpha = 0.25 

        # Previous command for smoothing
        self.q_prev = self.q.copy()

        # Red detection thresholds
        self.lower_red1 = np.array([0, 70, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 70, 50])
        self.upper_red2 = np.array([180, 255, 255])

        # Camera properties
        self.img_width = 640
        self.img_height = 480

        # Target
        self.target_visible = False
        self.target_x = 0
        self.target_y = 0

        # Target persistence
        self.frames_without_target = 0
        self.max_frames_lost = 8 

        # Sub/pub
        self.create_subscription(Image, '/franka/camera/image_raw', self.image_cb, 2)
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/franka/joint_group_position_controller/commands', 10)

        self.create_timer(0.05, self.control)  # 20 Hz

        self.get_logger().info("Simple Franka Tracker - ULTRA SMOOTH")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) | \
               cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        
        # Heavier morphology for more stable detection
        kernel = np.ones((7,7), np.uint8)  # Increased from 5x5
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                self.target_x = int(M['m10']/M['m00'])
                self.target_y = int(M['m01']/M['m00'])
                self.target_visible = True
                self.frames_without_target = 0
                return
        
        self.frames_without_target += 1
        if self.frames_without_target > self.max_frames_lost:
            self.target_visible = False

    def control(self):
        dq = np.zeros(7)

        if self.target_visible:
            # Compute error
            err_x = self.img_width/2 - self.target_x
            err_y = self.img_height/2 - self.target_y

            # Deadzone
            if abs(err_x) < self.deadzone_px:
                err_x = 0
            if abs(err_y) < self.deadzone_px:
                err_y = 0

            # Convert to joint changes
            dq[0] = -err_x * self.Kp_x
            dq[1] = err_y * self.Kp_y
            dq[2] = -err_y * self.Kp_y
            dq[3] = err_x * self.Kp_x
            dq[4] = (-err_y * self.Kp_y) * self.joint4_scale
            dq[5] = 0
            dq[6] = 0
        else:
            # SLOWER return to initial pose
            dq = (self.q_init - self.q) * 0.02  # Very gradual

        # Limit step size
        dq = np.clip(dq, -self.max_step, self.max_step)
        
        # DOUBLE SMOOTHING for ultra-smooth motion
        q_target = self.q + dq
        self.q = self.alpha * q_target + (1 - self.alpha) * self.q_prev
        self.q_prev = self.q.copy()

        # Enforce joint limits
        for i, (low, high) in enumerate(self.joint_limits):
            self.q[i] = np.clip(self.q[i], low, high)

        # Publish
        msg = Float64MultiArray()
        msg.data = self.q.tolist()
        self.pub.publish(msg)

        # Simplified logging
        if self.target_visible:
            self.get_logger().info(f"Tracking: err_x={err_x:.0f} err_y={err_y:.0f}", throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = SimpleFrankaTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
