import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from gpiozero import LED

class JudgementDay(Node):

    def __init__(self):
        super().__init__('judgement_node')

        self.is_autonomous = False

        self.publisher = self.create_publisher(Float32MultiArray, 'wheel_commands', 1)

        self.create_subscription(Bool, 'autobot_toggle', self.mode_callback, 1)
        self.create_subscription(Float32MultiArray, 'wheel_commands_driver', self.manual_callback, 1)
        self.create_subscription(Float32MultiArray, 'wheel_commands_tesla', self.auto_callback, 1)
        self.LED = LED(25)

    def mode_callback(self, msg):
        self.is_autonomous = msg.data
        state = "AUTONOMOUS" if self.is_autonomous else "MANUAL"
        self.get_logger().info(f"Wheel MUX mode: {state}")

    def manual_callback(self, msg):
        if not self.is_autonomous:
            # Assuming controller publishes [right, left] already scaled from -1 to 1
            self.publisher.publish(msg)
            self.LED.off()

    def auto_callback(self, msg):
        if self.is_autonomous:
            self.publisher.publish(msg)
            self.LED.on()

def main(args=None):
    rclpy.init(args=args)
    node = JudgementDay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()