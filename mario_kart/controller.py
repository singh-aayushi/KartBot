import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import subprocess
import re
import threading

class Controller(Node):
    
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'controller_inputs', 1)
        self.target_axes = {0, 1, 3, 4}
        self.axis_values = {i: 0 for i in self.target_axes}
        self.thread = threading.Thread(target=self.run_jstest)
        self.thread.start()
        
    def run_jstest(self):
        proc = subprocess.Popen(['jstest', '--event', '/dev/input/js0'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            for line in proc.stdout:
                match = re.search(r"Event: type 2, time \d+, number (\d+), value (-?\d+)", line)
                if match:
                    axis = int(match.group(1))
                    value = int(match.group(2))
                    if axis in self.target_axes:
                        self.axis_values[axis] = value
                        msg = Int32MultiArray()
                        msg.data = [self.axis_values[i] for i in sorted(self.axis_values)]
                        self.publisher_.publish(msg)
                        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            proc.terminate()
            
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()