import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import TransformException
import builtin_interfaces.msg

class DelayedTf2Broadcaster(Node):
    def __init__(self):
        super().__init__('delayed_tf2_broadcaster')
        
        # Parameters
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').get_parameter_value().double_value
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for looking up past transform
        self.timer = self.create_timer(0.1, self.broadcast_delayed_transform)
        
        self.get_logger().info(f'Delayed TF broadcaster started with delay: {self.delay} seconds')

    def broadcast_delayed_transform(self):
        try:
            # Get current time
            current_time = self.get_clock().now()
            
            # Calculate past time
            past_time = current_time - rclpy.time.Duration(seconds=self.delay)
            
            # Wait for transform to become available
            if self.tf_buffer.can_transform('world', 'turtle1_current', past_time):
                # Look up transform from the past
                transform = self.tf_buffer.lookup_transform(
                    'world',
                    'turtle1_current',
                    past_time
                )
                
                # Create new transform with current time but past pose
                delayed_tf = TransformStamped()
                delayed_tf.header.stamp = self.get_clock().now().to_msg()
                delayed_tf.header.frame_id = 'world'
                delayed_tf.child_frame_id = 'turtle1_delayed'
                
                # Copy the transform data from the past
                delayed_tf.transform = transform.transform
                
                self.tf_broadcaster.sendTransform(delayed_tf)
                
        except TransformException as ex:
            # Suppress the error messages after the first few
            pass
        except Exception as ex:
            self.get_logger().info(f'Error in delayed transform: {ex}')

def main():
    rclpy.init()
    node = DelayedTf2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()