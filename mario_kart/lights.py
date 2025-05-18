import rclpy
from rclpy.node import Node
from gpiozero import LED
from std_msgs.msg import Float32MultiArray
import time

class LightsNode(Node):
    def __init__(self):
        super().__init__('lights')
        self.LEDl = LED(20)
        self.LEDr = LED(21)
        
        # Subscriber to control lights
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_commands',
            self.listener_callback,
            1
        )

    def listener_callback(self, msg):
        cmdl = msg.data[0]
        cmdr = msg.data[1]
        if cmdl < cmdr:
            self.get_logger().info('Moving Left')
            self.LEDl.on()
            self.LEDr.off()
        elif cmdl > cmdr:
            self.get_logger().info('Moving right')
            self.LEDl.off()
            self.LEDr.on()
        else:
            if cmdl == 0:
                self.LEDl.on()
                self.LEDr.on()
            else:
                self.LEDl.off()
                self.LEDr.off()

def main(args=None):
    rclpy.init(args=args)
    node = LightsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()