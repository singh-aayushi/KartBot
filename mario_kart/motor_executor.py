#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 30 13:48:37 2025

@author: pi
"""

import rclpy
from rclpy.node import Node
import traceback
import signal

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from gpiozero import PhaseEnableMotor


class MotorExecutor(Node):

    def __init__(self):
        super().__init__('motor_executor')
        self.motor_left = PhaseEnableMotor(5,12)
        self.motor_right = PhaseEnableMotor(6,13)
        self.subscription = self.create_subscription(Float32MultiArray, 'wheel_commands', self.set_motor_command_both, 1)        

    def set_motor_command_both(self, msg_in): 
        motor_command_left = float(msg_in.data[0])
        self.get_logger().info('Received: %+5.3f' % msg_in.data[0])
        if motor_command_left >= 0:
            self.motor_left.forward(motor_command_left)
        else:
            self.motor_left.backward(-1*motor_command_left)
            
        motor_command_right = float(msg_in.data[1])
        self.get_logger().info('Received: %+5.3f' % msg_in.data[1])
        if motor_command_right >= 0:
            self.motor_right.forward(motor_command_right)
        else:
            self.motor_right.backward(-1*motor_command_right)    
            
    def stop_motor_both(self): 
        self.motor_left.stop()
        self.motor_right.stop()


def main(args=None):
    rclpy.init(args=args)

    motor_executor = MotorExecutor()

    try:
        # "spin" will block the program until an error occurs, e.g. keyboard break Ctrl+C. 
        rclpy.spin(motor_executor)
    except: 
        # When an error occurs, catch it with "except" and stop the motors
        motor_executor.stop_motor_both()
        motor_executor.get_logger().info('Stopping Motor') 
    finally:
        rclpy.shutdown()
        signal.signal(signal.SIGINT, lambda sig, frame: motor_executor.stop_motor_both())



# Section to start the execution.  
if __name__ == "__main__":
    main()
