#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from gazebo_msgs.msg import LinkStates

ROBOT_BASE = 'tricycle::base_link'   # spawned entity + link
OBSTACLE_PREFIX = 'obstacles::obstacle'  # obstacles::obstacle1..5
THRESH = 0.5

class CameraTrigger(Node):
    def __init__(self):
        super().__init__('camera_trigger')
        self.pub = self.create_publisher(Empty, '/camera/trigger', 10)
        self.sub = self.create_subscription(LinkStates, '/gazebo/link_states', self.cb, 10)
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz
        self.near = False
        self.get_logger().info(f"Triggering camera when any obstacle link < {THRESH} m")

    def cb(self, msg: LinkStates):
        # find robot base pose
        try:
            i_rb = msg.name.index(ROBOT_BASE)
        except ValueError:
            self.near = False
            return
        pr = msg.pose[i_rb].position

        # compute min distance to any obstacle link
        dmin = float('inf')
        for name, pose in zip(msg.name, msg.pose):
            if not name.startswith(OBSTACLE_PREFIX):
                continue
            po = pose.position
            d = math.hypot(pr.x - po.x, pr.y - po.y)  # planar distance
            if d < dmin:
                dmin = d

        self.near = (dmin < THRESH) if dmin < float('inf') else False

    def tick(self):
        if self.near:
            self.pub.publish(Empty())

def main():
    rclpy.init()
    node = CameraTrigger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
