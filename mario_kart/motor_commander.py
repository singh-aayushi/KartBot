#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Updated on 2025-01-30

@author: pi
"""

import rclpy
from rclpy.node import Node
import traceback

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class MotorCommander(Node):

    def __init__(self):
        super().__init__('motor_commander')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_commands', 1)
        self.subscriber_ = self.create_subscriber(F)

    def operate(self):
        msg = Float32MultiArray()
        try:
            while True:
                #cmdL = float(input('enter cmdL: ').strip())  
                #cmdR = float(input('enter cmdR: ').strip())
                cmd = input('enter cmd: ').strip()
                cmd = cmd.split()
                if len(cmd) > 1:
                    cmdL = float(cmd[0])
                    cmdR = float(cmd[1])
                else:
                    cmdL = float(cmd[0])
                    cmdR = float(cmd[0])
                msg.data = [cmdL, cmdR]
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing Left: %+5.3f' % msg.data[0])
                self.get_logger().info('Publishing Right: %+5.3f' % msg.data[1])
        except:
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)

    motor_commander_instance = MotorCommander()
    motor_commander_instance.operate()
    # NOTE that this instance has an infinite loop in it, so we don't have to "spin" the node to keep it from ending.
#    rclpy.spin(motor_commander_instance)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    