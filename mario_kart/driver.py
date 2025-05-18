import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

class Driver(Node):
    
    def __init__(self):
        super().__init__('driver')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_commands_driver', 1)
        self.subscriber_ = self.create_subscription(Float32MultiArray, 'controller_outputs', self.drive, 1)
        self.subscriber_boot = self.create_subscription(Bool, 'boost', self.update, 1)
        self.vmax = 0.5
        self.boost = True
    
    def update(self, msg_in):
        self.boost = msg_in.data
        if self.boost:
            self.vmax = 1.0
        else:
            self.vmax = 0.5
        
    def drive(self, msg_in):
        x = msg_in.data[0]
        y = msg_in.data[1]
        
        
        msg_out = Float32MultiArray()
        msg_out.data = [0.0, 0.0]
        
        if x < 0 and y == 0:
            msg_out.data = self.pivot_left(x)
        elif x > 0 and y == 0:
            msg_out.data = self.pivot_right(x)
        elif x == 0 and y < 0:
            msg_out.data = self.backward(y)
        elif x == 0 and y > 0:
            msg_out.data = self.forward(y)
        elif x != 0 and y != 0:
            msg_out.data = self.custom(x, y)
            
        msg_out.data = [i*self.vmax for i in msg_out.data]
        
        self.publisher_.publish(msg_out)
        
    def pivot_left(self, x):
        return [x, -x]
    
    def pivot_right(self, x):
        return [x, -x]
    
    def backward(self, y):
        return [y, y]
    
    def forward(self, y):
        return [y, y]
    
    def custom(self, x, y):
        vl = y + x
        vr = y - x
        max_v = max(abs(vl), abs(vr))
        if max_v > 1:
            vl /= max_v
            vr /= max_v
        return [vl, vr]
            
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()