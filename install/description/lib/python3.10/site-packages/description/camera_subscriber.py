#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # SensorData QoS (matches Gazebo camera)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to BOTH common topic names; whichever exists will deliver frames
        self.sub1 = self.create_subscription(Image, '/front_camera/image_raw',
                                             self.cb, sensor_qos)
        self.sub2 = self.create_subscription(Image, '/front_camera/image',
                                             self.cb, sensor_qos)

        self.get_logger().info('Camera subscriber up — watching /front_camera/image[_raw] with SensorData QoS')

    def cb(self, msg: Image):
        self.get_logger().info(
            f"Image on topic received: {msg.width}x{msg.height} (stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec})"
        )

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
