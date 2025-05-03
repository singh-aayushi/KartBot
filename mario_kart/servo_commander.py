#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Servo Command Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-16

servo_command_node.py
ROS Node to get User Input of servo microsecond commands (e.g. "/servo_command_0") and publish them to a topic to set servo movement in another node.  
"""

import rclpy
from rclpy.node import Node
import traceback 
# # Import the Servo library: 
# #from Adafruit_PWM_Servo_Driver import PWM
# import board
# from adafruit_pca9685 import PCA9685
# IMPORT the messages: 
from std_msgs.msg import Int32


class ServoCommander(Node):

    def __init__(self):
        super().__init__('servo_commander')
        self.publisher_ = self.create_publisher(Int32, 'servo_command_0', 1)
        
        #### CODE HERE ####
        # Add a Publisher for any more servos
        #### END CODE ####
      
        
    def operate(self):
        servo_command_0_msg = Int32()
        try:
            while True:
                servo_command_0 = int(input('Enter servo 0 command:  ').strip())
                servo_command_0_msg.data = servo_command_0
                self.publisher_.publish(servo_command_0_msg)
                self.get_logger().info('Publishing: %+5.3f' % servo_command_0_msg.data)
                #### CODE HERE ####
                # Add a input and publishing for any other servos
                #### END CODE ####


        except:
            traceback.print_exc()  
  
    
       
def main(args=None):
    rclpy.init(args=args)

    servo_commander_instance = ServoCommander()
    servo_commander_instance.operate()
        
    
if __name__ == '__main__':
    main()