import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32MultiArray
import time

class AutoBotToggle(Node):

    def __init__(self):
        super().__init__('autobot_toggle')

        # Konami Code
        self.konami_code = ["up", "up", "down", "down", "left", "right", "left", "right"]
        self.input_sequence = []
        self.sequence_time = []
        self.max_duration = 3.0  # seconds

        self.is_autonomous = False

        # Publishers and Subscribers
        self.mode_pub = self.create_publisher(Bool, 'autobot_toggle', 1)
        self.dpad_sub = self.create_subscription(String, 'dpad_direction', self.dpad_callback, 1)
        self.joy_sub = self.create_subscription(Int32MultiArray, 'controller_inputs', self.joy_callback, 1)

    def dpad_callback(self, msg: String):
        direction = msg.data.strip()
        now = time.time()

        if direction not in {"up", "down", "left", "right"}:
            return

        # Clear stale input sequence if first input is too old
        if self.sequence_time and (now - self.sequence_time[0] > self.max_duration):
            self.input_sequence.clear()
            self.sequence_time.clear()

        # Append new input
        self.input_sequence.append(direction)
        self.sequence_time.append(now)

        # Trim to max length
        if len(self.input_sequence) > len(self.konami_code):
            self.input_sequence.pop(0)
            self.sequence_time.pop(0)

        # Check for match
        if self.input_sequence == self.konami_code:
            duration = self.sequence_time[-1] - self.sequence_time[0]
            if duration <= self.max_duration:
                self.toggle_mode()
                self.input_sequence.clear()
                self.sequence_time.clear()

    def joy_callback(self, msg: Int32MultiArray):
        if not self.is_autonomous:
            return

        # Check for manual override input
        if any(abs(val) > 5000 for val in msg.data):  # Avoid noise
            self.get_logger().info("Manual override detected. Switching to MANUAL mode.")
            self.is_autonomous = False
            self.publish_mode(self.is_autonomous)

    def toggle_mode(self):
        self.is_autonomous = True
        self.publish_mode(self.is_autonomous)

    def publish_mode(self, mode: bool):
        msg = Bool()
        msg.data = mode
        self.mode_pub.publish(msg)
        state = "AUTONOMOUS" if mode else "MANUAL"
        self.get_logger().info(f"Mode changed: {state}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoBotToggle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()