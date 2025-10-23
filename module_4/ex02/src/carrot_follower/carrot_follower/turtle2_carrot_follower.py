import math
from geometry_msgs.msg import Twist, TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Turtle2CarrotFollower(Node):
    def __init__(self):
        super().__init__('turtle2_carrot_follower')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.follow_carrot)

    def follow_carrot(self):
        try:
            # Look up transform from turtle2 to carrot
            t = self.tf_buffer.lookup_transform(
                'turtle2',
                'carrot',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform turtle2 to carrot: {ex}')
            return

        # Create velocity command
        msg = Twist()
        
        # Linear velocity proportional to distance
        distance = math.sqrt(t.transform.translation.x**2 + t.transform.translation.y**2)
        msg.linear.x = 0.5 * distance
        
        # Angular velocity proportional to angle
        angle_to_carrot = math.atan2(t.transform.translation.y, t.transform.translation.x)
        msg.angular.z = 4.0 * angle_to_carrot

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = Turtle2CarrotFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
