import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class ArucoDistanceProcessor(Node):
    def __init__(self):
        super().__init__('aruco_distance_processor')
        self.subscription = self.create_subscription(
            String,
            'aruco',
            self.aruco_callback,
            1
        )
        
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'distance',
            1
        )

    def aruco_callback(self, msg):
        
        published = False
            
        data = msg.data.strip()
        if not data:
            return

        frames = data.split('ID:')
        for frame in frames:
            if not frame.strip():
                continue

            parts = frame.strip().split(',')
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            msg_out = Float32MultiArray()
            msg_out.data = [x, y, z]
            self.publisher.publish(msg_out)
            published = True
        
        if not published:
            msg_out = Float32MultiArray()
            msg_out.data = [0.0, 0.0, 0.0]
            self.publisher.publish(msg_out)
            

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDistanceProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()