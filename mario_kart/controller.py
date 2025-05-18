import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import time
import subprocess
import re
import threading

class Controller(Node):
    
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'controller_inputs', 1)
        self.dpad_string_pub = self.create_publisher(String, 'dpad_direction', 1)

        self.target_axes = {0, 1, 3, 4}  # analog sticks
        self.dpad_axes = {6, 7}         # D-pad axes

        self.axis_values = {i: 0 for i in self.target_axes}
        self.dpad_values = {i: 0 for i in self.dpad_axes}

        self.last_dpad_publish_time = {"up": 0, "down": 0, "left": 0, "right": 0}
        self.last_dpad_direction = {6: "", 7: ""}  # per-axis tracking
        self.dpad_cooldown = 0.2  # seconds

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

                    # Process analog sticks
                    if axis in self.target_axes:
                        self.axis_values[axis] = value
                        msg = Int32MultiArray()
                        msg.data = [self.axis_values[i] for i in sorted(self.axis_values)]
                        self.publisher_.publish(msg)

                    # Process D-pad (axes 6 and 7)
                    elif axis in self.dpad_axes:
                        direction = None
                        current_time = time.time()

                        if axis == 6:
                            if value == -32767:
                                direction = "left"
                            elif value == 32767:
                                direction = "right"
                        elif axis == 7:
                            if value == -32767:
                                direction = "up"
                            elif value == 32767:
                                direction = "down"

                        # Publish direction with cooldown
                        if direction:
                            last_time = self.last_dpad_publish_time[direction]
                            if current_time - last_time >= self.dpad_cooldown:
                                self.last_dpad_publish_time[direction] = current_time
                                msg = String()
                                msg.data = direction
                                self.dpad_string_pub.publish(msg)
                                self.last_dpad_direction[axis] = direction

                        # Clear last direction if released, but do not publish
                        elif value == 0:
                            self.last_dpad_direction[axis] = ""
                            
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
