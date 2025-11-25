#!/usr/bin/env python3
"""
Continuous Robot Movement Controller
Press keys to control the TurtleBot3 in real-time
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import threading
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.linear_speed = 0.22
        self.angular_speed = 2.0

        self.last_key = None
        self.running = True

    def publish_velocity(self):
        """Continuously publish velocity commands"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = self.linear_x
        msg.twist.angular.z = self.angular_z
        self.publisher.publish(msg)

    def move_forward(self):
        self.linear_x = self.linear_speed
        self.angular_z = 0.0
        print("↑ Moving Forward    ", end='\r', flush=True)

    def move_backward(self):
        self.linear_x = -self.linear_speed
        self.angular_z = 0.0
        print("↓ Moving Backward   ", end='\r', flush=True)

    def turn_left(self):
        self.linear_x = 0.0
        self.angular_z = self.angular_speed
        print("← Turning Left      ", end='\r', flush=True)

    def turn_right(self):
        self.linear_x = 0.0
        self.angular_z = -self.angular_speed
        print("→ Turning Right     ", end='\r', flush=True)

    def stop(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        print("■ Stopped           ", end='\r', flush=True)

    def increase_speed(self):
        self.linear_speed = min(1.0, self.linear_speed + 0.05)
        self.angular_speed = min(3.0, self.angular_speed + 0.2)
        print(f"++ Speed: {self.linear_speed:.2f} m/s   ", end='\r', flush=True)

    def decrease_speed(self):
        self.linear_speed = max(0.1, self.linear_speed - 0.05)
        self.angular_speed = max(0.5, self.angular_speed - 0.2)
        print(f"-- Speed: {self.linear_speed:.2f} m/s   ", end='\r', flush=True)

def get_key(settings):
    """Non-blocking key reading"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

def main():
    rclpy.init()
    mover = RobotMover()

    # Terminal settings
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    print("\n" + "="*60)
    print("  TURTLEBOT3 CONTINUOUS MOVEMENT CONTROLLER")
    print("="*60)
    print("\n  Controls:")
    print("    w = Forward ↑       s = Backward ↓")
    print("    a = Left ←          d = Right →")
    print("    SPACE = Stop        q = Quit")
    print("    + = Faster          - = Slower")
    print("\n  Press and hold keys for continuous movement!")
    print("="*60 + "\n")

    try:
        # Create executor in separate thread
        executor_thread = threading.Thread(target=rclpy.spin, args=(mover,), daemon=True)
        executor_thread.start()

        while mover.running:
            key = get_key(settings)

            if key:
                if key == 'w':
                    mover.move_forward()
                elif key == 's':
                    mover.move_backward()
                elif key == 'a':
                    mover.turn_left()
                elif key == 'd':
                    mover.turn_right()
                elif key == ' ':
                    mover.stop()
                elif key == '+' or key == '=':
                    mover.increase_speed()
                elif key == '-' or key == '_':
                    mover.decrease_speed()
                elif key == 'q':
                    print("\n\nQuitting...")
                    mover.stop()
                    time.sleep(0.2)
                    mover.running = False
                    break
                elif key == '\x03':  # Ctrl+C
                    break

            time.sleep(0.05)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        mover.stop()
        time.sleep(0.2)
        mover.destroy_node()
        rclpy.shutdown()
        print("\nController stopped.")

if __name__ == '__main__':
    main()
