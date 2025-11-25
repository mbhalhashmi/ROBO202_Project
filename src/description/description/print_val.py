#!/usr/bin/env python3

# Importing Libraries and Interfaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Create a class that inherits from rclpy Node class
class PrintValue(Node):
    def __init__(self):
# Call the parentt constructor with super()
        super().__init__('first_node')
        self.vel_value = None
        
        # subscribe to /cmd_vel
        self.cmdvel=self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback,10)
        self.timer=self.create_timer(3.0, self.timer_callback)

    def cmdvel_callback(self,msg):
        self.vel_value=msg

    def timer_callback(self):
        if self.vel_value:
            self.get_logger().info(f"Velocity Subscribed from cmd_vel topics: "
            f'linear.x={self.vel_value.linear.x}, angular.z={self.vel_value.angular.z}')

# Main function
def main(args=None):
    rclpy.init(args=args) # Initalise ROS@ communication
    node = PrintValue() # Create an object of the class
    rclpy.spin(node) # Start Node execution
    rclpy.shutdown() # Shutdown ROS2 communication

# Calling the main() function
if __name__=='__main__':
    main()
