import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import subprocess
import re
import threading

class ButtonController(Node):

    def __init__(self):
        super().__init__('button_controller')
        self.publisher_ = self.create_publisher(Int32, 'button_0_state', 1)
        self.thread = threading.Thread(target=self.run_jstest)
        self.thread.start()

    def run_jstest(self):
        proc = subprocess.Popen(['jstest', '/dev/input/js0'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            for line in proc.stdout:
                match = re.search(r'\b0:(on|off)\b', line)
                if match:
                    state = 1 if match.group(1) == 'on' else 0
                    msg = Int32()
                    msg.data = state
                    self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            proc.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = ButtonController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
