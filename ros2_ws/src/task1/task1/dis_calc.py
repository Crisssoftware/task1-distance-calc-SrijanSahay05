import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(
            Float32,
            '/turtle1/distance_from_origin',
            10)

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y
        
        distance = math.sqrt(x**2 + y**2)
        
        distance_msg = Float32()
        distance_msg.data = distance
        self.publisher.publish(distance_msg)

def main(args=None):
    rclpy.init(args=args)
    
    distance_subscriber = DistanceSubscriber()
    
    try:
        rclpy.spin(distance_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        distance_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
