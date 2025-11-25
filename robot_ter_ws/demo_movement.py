#!/usr/bin/env python3
"""
Demo: Make the robot move in a square pattern
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

def main():
    print("\n" + "="*50)
    print("  ROBOT MOVEMENT DEMO - SQUARE PATTERN")
    print("="*50 + "\n")

    rclpy.init()
    node = Node('demo_movement')
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    # Wait for connection
    time.sleep(1)
    print("Starting demo in 2 seconds...")
    time.sleep(2)

    def move_forward(duration, speed=0.2):
        """Move forward for specified duration"""
        print(f"→ Moving forward for {duration} seconds at {speed} m/s")
        msg = TwistStamped()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = speed

        start = time.time()
        while time.time() - start < duration:
            msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)
            time.sleep(0.1)

        # Stop
        msg.twist.linear.x = 0.0
        publisher.publish(msg)

    def turn(duration, angular_speed=1.0):
        """Turn for specified duration"""
        print(f"↻ Turning for {duration} seconds at {angular_speed} rad/s")
        msg = TwistStamped()
        msg.header.frame_id = 'base_footprint'
        msg.twist.angular.z = angular_speed

        start = time.time()
        while time.time() - start < duration:
            msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)
            time.sleep(0.1)

        # Stop
        msg.twist.angular.z = 0.0
        publisher.publish(msg)

    try:
        # Move in square pattern
        for i in range(4):
            print(f"\n[Side {i+1}/4]")
            move_forward(2.0)  # Move forward 2 seconds
            time.sleep(0.5)    # Pause
            turn(1.57)         # Turn 90 degrees (~1.57 seconds at 1 rad/s)
            time.sleep(0.5)    # Pause

        print("\n✓ Demo complete! Robot moved in a square pattern.")
        print("Check the Gazebo window to see the final position.\n")

    except KeyboardInterrupt:
        print("\n\nDemo interrupted!")

    finally:
        # Final stop
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        publisher.publish(msg)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
