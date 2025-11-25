#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraRecordingAndSafety(Node):
    def __init__(self):
        super().__init__('camera_safety_node')

        # 1. Camera Subscription (Best Effort QoS for Gazebo)
        self.sub_cam = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data)

        # 2. Teleop Interception
        self.sub_teleop = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.cmd_callback, 10)

        # 3. Output to Robot
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.br = CvBridge()
        self.obstacle_detected = False
        self.get_logger().info("📸 Safety Node Launched. Waiting for camera data...")

    # --- NEW HELPER FUNCTION ---
    def is_object_closer_than_0_5m(self, cv_image):
        # 1. Convert to Grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 2. Crop to the center (ROI) to avoid floor/sky
        height, width = gray_image.shape
        y_start = int(height * 0.3)
        y_end   = int(height * 0.7)
        x_start = int(width * 0.3)
        x_end   = int(width * 0.7)
        
        roi = gray_image[y_start:y_end, x_start:x_end]
        
        # 3. Calculate "Busyness" (Standard Deviation)
        disturbance_score = np.std(roi)
        
        # Debug logging (Optional: remove if too noisy)
        self.get_logger().info(f"Disturbance Score: {disturbance_score:.2f}", throttle_duration_sec=0.5)
        
        # 4. The Check
        # If score > 2.0, it's not empty gray space -> Object is close.
        if disturbance_score > 2.0:
            return True
        else:
            return False

    def image_callback(self, data):
        try:
            # Convert ROS message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

            # --- CHECK DISTANCE ---
            if self.is_object_closer_than_0_5m(current_frame):
                
                # Only take action if this is a NEW detection
                if not self.obstacle_detected:
                    self.get_logger().warn("🛑 STOP! Object within 0.5m")
                    
                    # --- TAKE PHOTO ---
                    filename = "obstacle_detected.jpg"
                    cv2.imwrite(filename, current_frame)
                    self.get_logger().info(f"📸 SNAPSHOT SAVED: {filename}")
                    
                self.obstacle_detected = True
                self.stop_robot()
            else:
                if self.obstacle_detected:
                    self.get_logger().info("✅ Path Clear")
                self.obstacle_detected = False

        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

    def cmd_callback(self, msg):
        if self.obstacle_detected:
            self.stop_robot()
        else:
            self.pub_cmd.publish(msg)

    def stop_robot(self):
        stop_msg = Twist()
        self.pub_cmd.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraRecordingAndSafety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()