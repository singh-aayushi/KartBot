#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Sensors to Motor Command Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-17

sensors_to_motor_command_node.py
ROS Node to accept commands of "motor_command_left" and "motor_command_right" and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

import rclpy
from rclpy.node import Node
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Float32MultiArray, Int32MultiArray



class SensorsToMotorCommands(Node):
    def __init__(self): 
        super().__init__('sensors_to_motor_commands')
            
        # Set up callable Publishers and messages
        self.pub_motor_command = self.create_publisher(Float32MultiArray,'wheel_commands',1)
        
        # Set up a Subscriber to sensors
        self.sub_sensors_light = self.create_subscription(Int32MultiArray,'light_sensors_proc',self.light_sensors_to_motor_commands,1)
        

    # Callback for when a message comes in 
    def light_sensors_to_motor_commands(self,msg_in):
        
        default_speed = 0.1
    
        # unpack the message
        A0, A1, A2 = msg_in.data # where A0 is the left, A1 middle, A2 right
        
        if A1 == 1:
            forward = True
        else:
            forward = False
            
        motor_command_msg = Float32MultiArray()
        
        if forward:
            motor_command_msg.data = [default_speed, default_speed]
            self.get_logger().info("Moving forward.")
        else:
            motor_command_msg.data = [0, 0]
            self.get_logger().info("Stopping motors.")
            
        self.pub_motor_command.publish(motor_command_msg)
 

    # here a safety function: if an error happens, stop the motors. 
    def send_stop_commands(self): 
        stop_msg = Float32MultiArray()
        stop_msg.data = [0, 0]
        self.pub_motor_command(stop_msg.data)
        self.get_logger().info("Stopping motors.")
        
    
def main(args=None): 
    rclpy.init(args=args)    
    
    sensors_to_motor_cmd_instance = SensorsToMotorCommands()
    
    try: 
        rclpy.spin(sensors_to_motor_cmd_instance)
    except: 
        traceback.print_exc()
        sensors_to_motor_cmd_instance.send_stop_commands()
        
    rclpy.shutdown()
    
    
# Section to start the execution if called from a regular user, with Exception handling. 
if __name__ == "__main__": 
    main()
