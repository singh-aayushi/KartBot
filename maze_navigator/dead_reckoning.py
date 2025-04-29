import rclpy
from rclpy.node import Node
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Float32MultiArray
import numpy as np


class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')
        
        self.encoder_sub = self.create_subscription(Float32MultiArray, 'encoders_proc', self.position, 1)
        self.position_pub = self.create_publisher(Float32MultiArray, 'position', 1)
        
        self.x = 0
        self.y = 0
        self.theta = 0
        self.l_encoder = 0
        self.r_encoder = 0
        
        self.wheel_diameter = 0.05
        self.wheel_width = 0.15
        
    def position(self, msg_in):
        new_l_encoder, new_r_encoder = msg_in.data
        delta_l_encoder = new_l_encoder - self.l_encoder
        delta_r_encoder = new_r_encoder - self.r_encoder
        
        delta_sl = delta_l_encoder*np.pi*self.wheel_diameter
        delta_sr = delta_r_encoder*np.pi*self.wheel_diameter
        
        delta_s = (delta_sl + delta_sr)/2
        
        delta_theta = (delta_sr - delta_sl)/self.wheel_width
        
        self.x = self.x - delta_s*np.sin(self.theta + delta_theta/2)
        self.y = self.y + delta_s*np.cos(self.theta + delta_theta/2)
        self.theta = self.theta + delta_theta
        
        msg = Float32MultiArray()
        msg.data = [self.x, self.y]
        self.position_pub.publish(msg)
        
        self.l_encoder = new_l_encoder
        self.r_encoder = new_r_encoder
            
def main(args=None): 
    rclpy.init(args=args)    
    
    dead_reckoning_instance = DeadReckoning()
    
    try: 
        rclpy.spin(dead_reckoning_instance)
    except: 
        traceback.print_exc()
        
    rclpy.shutdown()
    
    
# Section to start the execution if called from a regular user, with Exception handling. 
if __name__ == "__main__": 
    main()
            
            