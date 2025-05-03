#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Servo Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2024-09-24

servo_node.py
ROS Node to set servo pulses received on input topics (e.g. "/servo_command_0"). 
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

# Duplicate all the functions from "servo_examples"
#from Adafruit_PWM_Servo_Driver import PWM
import board
from adafruit_pca9685 import PCA9685
import numpy as np
import time

class servos(): 
    def __init__(self): 
        # Create the I2C bus interface.
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        
        # PWM Controller set-up
        self.servo_pulse_frequency = 50
        self.servo_pulse_duration_us = 1.0e6 / self.servo_pulse_frequency
        self.servo_pulse_width_to_count_multiplier = 1.0 / self.servo_pulse_duration_us * 65535
        self.servo_us_range = 2000
            
        # Initialize the PCA9685
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        
        # Set all channels to 0 initially
        for i in range(16):
            self.pca.channels[i].duty_cycle = 0
    
    # Command servo on the given servo number
    def command_servo(self, servo_number, pulse_width_us):
        pulse_width_count = int(round(pulse_width_us * self.servo_pulse_width_to_count_multiplier))
        self.pca.channels[servo_number].duty_cycle = pulse_width_count
    
    # Servo angle interpolation function
    def interp_servo(self, angle_range, us_range, angle):
        return np.interp(angle, angle_range, us_range)
    
    # Shut down all servos
    def shutdown_servos(self):
        for i in range(16):
            self.command_servo(i, 0)

class ServoExecutor(Node):
    def __init__(self):
        super().__init__('servo_executor')
        self.subscription = self.create_subscription(Int32, 
                                                     'button_0_state', 
                                                     self.command_servo_0, 1)     
        self.servos = servos()

    def command_servo_0(self, msg_in): 
        cmd_0 = msg_in.data
        if cmd_0 == 1:
            self.get_logger().info(f'Received command: {cmd_0}')
            self.servos.command_servo(1, 500)
            time.sleep(0.5)
            self.servos.command_servo(1, 2500)
            



# =============================================================================
#   # Main function
# =============================================================================
def main(args=None):
    rclpy.init(args=args)

    servo_executor = ServoExecutor()

    try:
        # "spin" will block the program until an error occurs, e.g. keyboard break Ctrl+C. 
        rclpy.spin(servo_executor)
    except: 
        # When an error occurs, catch it with "except" and stop the motors
        servo_executor.servos.shutdown_servos() 
        servo_executor.get_logger().info('Stopping Motor') 

    rclpy.shutdown()

# =============================================================================

    
    
    
# Startup stuff. 
if __name__ == '__main__':
    try: 
        main()
    except: 
        traceback.print_exc()

    # Shut down here also, just in case. 
    s = servos()
    s.shutdown_servos()    # to shut it dow



