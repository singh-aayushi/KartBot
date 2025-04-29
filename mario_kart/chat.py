#!/usr/bin/env python3
# keyboard_controller.py
# ROS2 node to control robot via keyboard arrow keys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

# Key codes for arrows
KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT = '\x1b[D'
KEY_SPACE = ' '
KEY_Q = 'q'

# Terminal settings helper
class KeyboardReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self):
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(3)  # arrows are 3-byte sequences
        return None

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_commands', 1)
        self.get_logger().info('KeyboardController started. Use arrow keys to drive, space to stop, q to quit.')

    def run(self):
        msg = Float32MultiArray()
        with KeyboardReader() as reader:
            try:
                while rclpy.ok():
                    key = reader.read_key()
                    throttle = 0.0
                    steer = 0.0
                    if key == KEY_UP:
                        throttle = 0.5
                    elif key == KEY_DOWN:
                        throttle = -0.5
                    elif key == KEY_LEFT:
                        steer = 0.5
                    elif key == KEY_RIGHT:
                        steer = -0.5
                    elif key == KEY_SPACE:
                        throttle = 0.0
                        steer = 0.0
                    elif key and key.endswith(KEY_Q):
                        break
                    else:
                        continue

                    left = throttle + steer
                    right = throttle - steer
                    # clamp
                    left = max(min(left, 1.0), -1.0)
                    right = max(min(right, 1.0), -1.0)

                    msg.data = [left, right]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing wheel_commands: left={left:.2f}, right={right:.2f}')
            except KeyboardInterrupt:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
