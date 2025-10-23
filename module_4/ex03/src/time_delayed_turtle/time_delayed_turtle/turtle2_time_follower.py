import math
from geometry_msgs.msg import Twist, TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Turtle2TimeFollower(Node):
    def __init__(self):
        super().__init__('turtle2_time_follower')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for turtle2 velocity commands
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.follow_delayed_pose)
        
        self.get_logger().info('Turtle2 time follower started')

    def follow_delayed_pose(self):
        try:
            # Look up transform from turtle2 to the delayed turtle1 pose
            transform = self.tf_buffer.lookup_transform(
                'turtle2',
                'turtle1_delayed',
                rclpy.time.Time()
            )
            
            # Calculate distance and angle to target
            target_x = transform.transform.translation.x
            target_y = transform.transform.translation.y
            distance = math.sqrt(target_x**2 + target_y**2)
            angle_to_target = math.atan2(target_y, target_x)
            
            # Create velocity command
            msg = Twist()
            
            # Linear velocity (proportional to distance with saturation)
            msg.linear.x = min(0.8 * distance, 2.0)
            
            # Angular velocity (proportional to angle)
            msg.angular.z = 4.0 * angle_to_target

            self.publisher.publish(msg)
            
        except TransformException as ex:
            # Suppress the error messages after the first few
            pass

def main():
    rclpy.init()
    node = Turtle2TimeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()