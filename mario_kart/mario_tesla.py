import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MarioTesla(Node):

    def __init__(self):
        super().__init__('mario_tesla')

        # Subscribes to ArUco marker position (relative to camera)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'distance',
            self.callback,
            1)

        # Publishes wheel speeds [left_speed, right_speed]
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'wheel_commands_tesla',
            1)

        # Controller gains
        self.k_linear = 0.25   # Forward gain
        self.k_angular = 0.5  # Steering gain
        self.target_distance = 0.1  # Stop within 30 cm

    def callback(self, msg):
        x = msg.data[0]  # left-right
        y = msg.data[1]  # up-down (ignored)
        z = msg.data[2]  # forward distance
        z = max(0.0, z)

        # Stop if within threshold
        if z < 0.1:
            self.publish_wheel_speeds(0.0, 0.0)
            self.get_logger().info('Reached target.')
            return

        # Full speed when far, slow down when close
        if z >= 0.5:
            linear_speed = 0.5
        else:
            # Linearly scale speed between 0.0 to 1.0 as z goes from 0.1 to 0.5
            linear_speed = max(0.1, z / 0.5)  # Prevent too slow motion

        # Proportional steering correction
        angular_correction = self.k_angular * x

        # Compute wheel speeds
        left_speed = linear_speed - angular_correction
        right_speed = linear_speed + angular_correction

        # Clamp wheel speeds to [-1.0, 1.0]
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        self.publish_wheel_speeds(left_speed, right_speed)


    def publish_wheel_speeds(self, left, right):
        msg = Float32MultiArray()
        msg.data = [-left, -right]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MarioTesla()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
