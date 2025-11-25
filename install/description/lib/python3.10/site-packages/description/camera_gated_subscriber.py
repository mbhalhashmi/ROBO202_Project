#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan

def sensor_qos(depth=10):
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )

class CameraGatedSubscriber(Node):
    def __init__(self):
        super().__init__('camera_gated_subscriber')

        # QoS to match Gazebo sensors
        qos = sensor_qos()

        # Subscribe to camera (support both common names)
        self.sub_img_raw = self.create_subscription(
            Image, '/front_camera/image_raw', self.image_cb, qos)
        self.sub_img      = self.create_subscription(
            Image, '/front_camera/image', self.image_cb, qos)

        # Subscribe to Lidar
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos)

        # gate parameters
        self.trigger_distance = 0.5  # meters
        self.window_deg = 45.0       # +/- degrees around forward
        self.latest_front_min = None
        self.have_scan = False

        self.get_logger().info(
            "Gated camera logger running. Will print images ONLY if front min range < 0.5 m."
        )

    # keep latest scan
    def scan_cb(self, msg: LaserScan):
        # compute front-sector indices (+/- window around 0° direction)
        window_rad = math.radians(self.window_deg)
        # angle 0 is straight ahead; compute indices for [-window,+window]
        start_angle = -window_rad
        end_angle = window_rad

        # map angles to indices
        def angle_to_index(angle):
            idx = int(round((angle - msg.angle_min) / msg.angle_increment))
            return max(0, min(idx, len(msg.ranges) - 1))

        i0 = angle_to_index(start_angle)
        i1 = angle_to_index(end_angle)
        if i1 < i0:
            i0, i1 = i1, i0

        sector = msg.ranges[i0:i1 + 1] if i1 >= i0 else []

        # sanitize values: keep only finite & within sensor range
        valid = []
        rmin = msg.range_min if msg.range_min > 0.0 else 0.0
        rmax = msg.range_max if msg.range_max > 0.0 else float('inf')
        for r in sector:
            if r is not None and math.isfinite(r) and rmin <= r <= rmax:
                valid.append(r)

        self.latest_front_min = min(valid) if valid else None
        self.have_scan = True

    # gate images based on latest scan
    def image_cb(self, msg: Image):
        if not self.have_scan:
            # Don’t spam; only note once in a while if needed
            return

        d = self.latest_front_min
        if d is not None and d < self.trigger_distance:
            self.get_logger().info(
                f"Image {msg.width}x{msg.height} (t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}) "
                f"— front min range: {d:.2f} m < {self.trigger_distance:.2f} m (LOGGED)"
            )
        # else: silently ignore frames when outside 0.5 m

def main():
    rclpy.init()
    node = CameraGatedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
