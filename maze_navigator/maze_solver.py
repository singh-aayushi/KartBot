#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import time

class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        
        self.busy = False # Flag to ignore sensor input during motion

        # Motion parameters 
        self.linear_speed = 0.1       
        self.turn_speed = 0.1     
        self.forward_time = 0.5   
        self.turn_time = 0.75      

        # Subscriber and publisher
        self.create_subscription(
            Int32MultiArray,
            'light_sensors_proc',
            self.sensor_callback,
            1
        )
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            'wheel_commands',
            1
        )

        self.get_logger().info('MazeSolver ready')

    def sensor_callback(self, msg):

        # Ignore sensor updates while executing a motion
        if self.busy:
            return

        # 0 = wall, 1 = free
        left, center, right = msg.data

        # Forward if center is free and both sides are walls
        if center == 1 and left == 0 and right == 0:
            self.move_forward()

        # Turn left ; all other sides blocked
        elif left == 1 and center == 0 and right == 0:
            self.turn_left()
            self.move_forward()

        # Turn right ; all other sides blocked
        elif right == 1 and center == 0 and left == 0:
            self.turn_right()
            self.move_forward()

        # If center free with any side free, move forward
        elif center == 1:
            self.move_forward()

        # If left and right free, take left
        elif left == 1 and right == 1:
            self.turn_left()
            self.move_forward()

        # Dead end: U-turn
        else:
            self.turn_left()
            self.turn_left()
            self.move_forward()

    def _publish(self, speeds):
        msg = Float32MultiArray()
        msg.data = speeds
        self.cmd_pub.publish(msg)

    def move_forward(self):
        self.busy = True
        self._publish([self.linear_speed, self.linear_speed])
        time.sleep(self.forward_time)
        self._stop_and_resume()

    def turn_left(self):
        self.busy = True
        self._publish([-self.turn_speed, self.turn_speed])
        time.sleep(self.turn_time)
        self._stop_and_resume()

    def turn_right(self):
        self.busy = True
        self._publish([self.turn_speed, -self.turn_speed])
        time.sleep(self.turn_time)
        self._stop_and_resume()

    def _stop_and_resume(self):
        self._publish([0.0, 0.0])
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()