#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Subscribe to the image topic published by Gazebo camera plugin
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',          # must match plugin remapping
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.frame_count = 0
        self.get_logger().info('Subscribed to /camera/image_raw')

    def image_callback(self, msg):
        self.frame_count += 1
        self.get_logger().info(
            f"Frame {self.frame_count}: "
            f"size=({msg.width}x{msg.height}), "
            f"encoding={msg.encoding}, "
            f"timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
