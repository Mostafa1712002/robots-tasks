#!/usr/bin/env python3
"""
Simple Robot Movement Control Script
Use arrow keys or WASD to control the TurtleBot3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.2
        self.angular_speed = 1.0

    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.publisher.publish(msg)
        print("↑ Moving Forward")

    def move_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.publisher.publish(msg)
        print("↓ Moving Backward")

    def turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)
        print("← Turning Left")

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.publisher.publish(msg)
        print("→ Turning Right")

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)
        print("■ Stopped")

    def increase_speed(self):
        self.linear_speed += 0.1
        self.angular_speed += 0.2
        print(f"++ Speed increased: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}")

    def decrease_speed(self):
        self.linear_speed = max(0.1, self.linear_speed - 0.1)
        self.angular_speed = max(0.2, self.angular_speed - 0.2)
        print(f"-- Speed decreased: Linear={self.linear_speed:.1f}, Angular={self.angular_speed:.1f}")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    controller = RobotController()

    print("\n" + "="*50)
    print("  TurtleBot3 Robot Controller")
    print("="*50)
    print("\nControls:")
    print("  w/↑ = Forward")
    print("  s/↓ = Backward")
    print("  a/← = Turn Left")
    print("  d/→ = Turn Right")
    print("  SPACE = Stop")
    print("  + = Increase Speed")
    print("  - = Decrease Speed")
    print("  q = Quit")
    print("="*50 + "\n")

    try:
        while True:
            key = get_key()

            if key == 'w' or key == '\x1b[A':  # w or up arrow
                controller.move_forward()
            elif key == 's' or key == '\x1b[B':  # s or down arrow
                controller.move_backward()
            elif key == 'a' or key == '\x1b[D':  # a or left arrow
                controller.turn_left()
            elif key == 'd' or key == '\x1b[C':  # d or right arrow
                controller.turn_right()
            elif key == ' ':  # space
                controller.stop()
            elif key == '+' or key == '=':
                controller.increase_speed()
            elif key == '-' or key == '_':
                controller.decrease_speed()
            elif key == 'q':
                print("\nQuitting...")
                controller.stop()
                break
            elif key == '\x03':  # Ctrl+C
                break

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
