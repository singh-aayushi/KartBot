import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time
from gpiozero import LED

class BoostNode(Node):
    def __init__(self):
        super().__init__('boost_node')
        self.publisher = self.create_publisher(Bool, 'boost', 1)
        self.subscriber = self.create_subscription(Int32, 'button_2_state', self.callback, 1)
        
        self.boost = True
        self.boostdur = 2.0
        self.boostcd = 5.0
        self.lastboost = 0.0
        
        self.LED = LED(26)

    def callback(self, msg):
        
        cmd = msg.data
        
        now = time.time()
        
        if self.boost and cmd == 1:
            if self.lastboost == 0.0:
                self.lastboost = now
        if (now - self.lastboost) > self.boostdur:
            self.boost = False
        if (now - self.lastboost) > (self.boostdur + self.boostcd) and cmd == 1:
            self.boost = True
            self.lastboost = now
            
        if self.boost:
            self.LED.on()
        else:
            if (now - self.lastboost) > (self.boostdur + self.boostcd):
                self.LED.on()
            else:
                self.LED.off()
                
        msg_out = Bool()
        msg_out.data = self.boost
        self.publisher.publish(msg_out)
                    
def main(args=None):
    rclpy.init(args=args)
    node = BoostNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()