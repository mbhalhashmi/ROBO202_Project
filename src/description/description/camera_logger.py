#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraLogger(Node):
    def __init__(self):
        super().__init__('camera_logger')
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        
        # Define a threshold for the total sum of pixel values.
        # This value needs to be tuned empirically: it should be higher than
        # the sum of a blank image but lower than an image with a small object.
        # For a 640x480 R8G8B8 image (approx 921,600 bytes), a sum of 100,000 
        # is a reasonable starting point for a colored object to be visible.
        self.pixel_sum_threshold = 100 
        self.get_logger().info("Camera Logger initialized. Only logging images where total pixel sum > 100000.")


    def cb(self, msg: Image):
        # 1. Get the raw image data (a byte array)
        raw_data = msg.data
        
        # 2. Calculate the total sum of all byte values in the image data.
        # Python's built-in sum() works on byte arrays, treating each byte as an integer value (0-255).
        total_pixel_sum = sum(raw_data)
        
        # 3. Check if the sum exceeds the threshold
        if total_pixel_sum > self.pixel_sum_threshold:
            # The image contains significant visible content (an obstacle within 0.5m)
            stamp = f"{msg.header.stamp.sec}.{str(msg.header.stamp.nanosec).zfill(9)}"
            self.get_logger().info(f"✅ Image CAPTURED: Obstacle visible. Total Pixel Sum: {total_pixel_sum} (stamp={stamp})")
        else:
            # Image is mostly empty (obstacle is outside the 0.5m clip range)
            self.get_logger().info(f"❌ Image Ignored: Obstacle outside 0.5m clip. Total Pixel Sum: {total_pixel_sum}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()