import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class ControllerProcessor(Node):
    
    def __init__(self):
        super().__init__('controller_processor')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'controller_outputs', 1)
        self.subscriber_ = self.create_subscription(Int32MultiArray, 'controller_inputs', self.processor, 1)
        
        #self.axes_mappings = {lx: 0, ly: 1, rx: 2, ry: 3}
        
        # calibration
        
        self.range = 32767
        
        self.activation = 0.05
        
        self.joylx = 0
        self.joyly = 0
        
        self.joyrx = 0
        self.joyry = 0
        
    def processor(self, msg_in):
        data = [i/self.range for i in msg_in.data]
        self.joylx = data[0]
        self.joyly = data[1]
        self.joyrx = data[2]
        self.joyry = data[3]
        
        msg_out = Float32MultiArray()
        msg_out.data = [0.0, 0.0]
        
        if abs(self.joylx) > self.activation:
            msg_out.data[0] = self.joylx
            
        if abs(self.joyry) > self.activation:
            msg_out.data[1] = self.joyry
            
        self.publisher_.publish(msg_out)
        
            
def main(args=None):
    rclpy.init(args=args)
    node = ControllerProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()