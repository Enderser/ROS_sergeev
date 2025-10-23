import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Current target
        self.current_target = 'carrot1'
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        # Subscribers
        self.target_sub = self.create_subscription(
            String,
            '/current_target_name',
            self.target_callback,
            10
        )
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Turtle controller started')

    def target_callback(self, msg):
        old_target = self.current_target
        self.current_target = msg.data
        if old_target != self.current_target:
            self.get_logger().info(f'Switched to target: {self.current_target}')

    def control_loop(self):
        try:
            # Look up transform from turtle2 to current target
            transform = self.tf_buffer.lookup_transform(
                'turtle2',
                self.current_target,
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
            msg.linear.x = min(0.5 * distance, 2.0)
            
            # Angular velocity (proportional to angle)
            msg.angular.z = 4.0 * angle_to_target

            self.cmd_vel_pub.publish(msg)
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform turtle2 to {self.current_target}: {ex}')

def main():
    rclpy.init()
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()