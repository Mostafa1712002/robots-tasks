#!/usr/bin/env python3
"""
Simple test script to move the robot
This will make the robot move forward for 2 seconds, then stop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main():
    print("Initializing ROS2...")
    rclpy.init()
    node = Node('test_movement')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # Wait for publisher to be ready
    time.sleep(1)

    # Move forward
    print("Moving forward for 2 seconds...")
    msg = Twist()
    msg.linear.x = 0.2  # 0.2 m/s forward

    for i in range(20):  # 2 seconds at 10Hz
        publisher.publish(msg)
        time.sleep(0.1)

    # Stop
    print("Stopping...")
    msg = Twist()
    publisher.publish(msg)

    print("Test complete!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
