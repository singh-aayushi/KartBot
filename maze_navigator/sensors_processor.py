#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: sensors_processor node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2024-10-03

sensors_processor.py
ROS2 Node to a receive sensor data on a topic ("sensors_A0" or other) and process it, then publish the processed version to another topic ("sensors_A0_processed" or similar)  
"""


import rclpy
from rclpy.node import Node
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32


class SensorsProcessor(Node): 
    def __init__(self): 
        super().__init__('sensors_processor')
        
        # First create one or more publishers for the topics that will hold the processed data. 
        self.publisher_light_sensors = self.create_publisher(Int32MultiArray, 'light_sensors_proc', 1)
        self.publisher_encoders = self.create_publisher(Float32MultiArray, 'encoders_proc', 1)
        
        # Also create Subscriptions with callbacks to handle each of the inputs 
        self.subscriber_A0 = self.create_subscription(Int32, 'sensors_A0', self.cal_and_pub_A0, 1)
        self.subscriber_A1 = self.create_subscription(Int32, 'sensors_A1', self.cal_and_pub_A1, 1)
        self.subscriber_A2 = self.create_subscription(Int32, 'sensors_A2', self.cal_and_pub_A2, 1)
        self.subscriber_E0 = self.create_subscription(Int32, 'sensors_E0', self.cal_and_pub_E0, 1)
        self.subscriber_E1 = self.create_subscription(Int32, 'sensors_E1', self.cal_and_pub_E1, 1)
        
        # aggregate the light and dark sensors
        self.A0 = None
        self.A1 = None
        self.A2 = None
        
        # aggreagete the light and dark sensors
        self.E0 = None
        self.E1 = None
        
        
        # Callback function, which will be called with incoming message data when messages are received by the Subscriber above. 
    def cal_and_pub_A0(self, msg_in): 
        self.A0 = self.light_level(msg_in)
        self.try_publish_light()
        
    def cal_and_pub_A1(self, msg_in):
        self.A1 = self.light_level(msg_in)
        self.try_publish_light()
        
    def cal_and_pub_A2(self, msg_in):
        self.A2 = self.light_level(msg_in)
        self.try_publish_light()

    def light_level(self, msg_in):
        if msg_in.data > 0:
            light = 1
        else:
            light = 0
        return light
        
    def try_publish_light(self):
        if self.A0 is not None and self.A1 is not None and self.A2 is not None:
            msg = Int32MultiArray()
            msg.data = [self.A0, self.A1, self.A2]
            self.publisher_light_sensors.publish(msg)
            self.A0 = None
            self.A1 = None
            self.A2 = None
            
    def cal_and_pub_E0(self, msg_in):
        self.E0 = self.encoder_count(msg_in)
        self.try_publish_encoder()
        
    def cal_and_pub_E1(self, msg_in):
        self.E1 = self.encoder_count(msg_in)
        self.try_publish_encoder()
        
    def encoder_count(self, msg_in):
        count_per_rev = 12
        N = 120
        total_counts_per_wheel_turn = count_per_rev*N
        return msg_in.data/total_counts_per_wheel_turn # this returns the number of wheel revolutions
    
    def try_publish_encoder(self):
        if self.E0 is not None and self.E1 is not None:
            msg = Float32MultiArray()
            msg.data = [self.E0, self.E1]
            self.publisher_encoders.publish(msg)
            self.E0 = None
            self.E1 = None

def main(args=None):
    rclpy.init(args=args)
    sensors_processor_instance = SensorsProcessor()
    try:
        rclpy.spin(sensors_processor_instance)
    except : 
        traceback.print_exc()

    

if __name__ == '__main__':
    main() 
    