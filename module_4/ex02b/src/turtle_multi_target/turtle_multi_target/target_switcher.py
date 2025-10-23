import math
import threading
import select
import sys
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import TransformException

class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')
        
        # Parameters
        self.declare_parameter('switch_threshold', 1.0)
        self.switch_threshold = self.get_parameter('switch_threshold').get_parameter_value().double_value
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Target management
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target_index = 0
        self.current_target = self.targets[self.current_target_index]
        
        # Keyboard input
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Publishers
        self.target_pub = self.create_publisher(String, '/current_target_name', 10)
        
        # Timer for broadcasting transforms
        self.timer = self.create_timer(0.1, self.broadcast_all_transforms)
        
        # Timer for automatic switching
        self.switch_timer = self.create_timer(0.5, self.check_auto_switch)
        
        self.angle_offset = 0.0
        
        self.get_logger().info(f'Target switcher started. Initial target: {self.current_target}')
        self.get_logger().info('Press "n" to manually switch targets')

    def broadcast_all_transforms(self):
        current_time = self.get_clock().now().to_msg()
        self.angle_offset += 0.1
        
        # Broadcast carrot1 (rotating around turtle1)
        try:
            carrot1_tf = TransformStamped()
            carrot1_tf.header.stamp = current_time
            carrot1_tf.header.frame_id = 'turtle1'
            carrot1_tf.child_frame_id = 'carrot1'
            
            # Rotating carrot around turtle1
            radius = 2.0
            carrot1_tf.transform.translation.x = radius * math.cos(self.angle_offset)
            carrot1_tf.transform.translation.y = radius * math.sin(self.angle_offset)
            carrot1_tf.transform.translation.z = 0.0
            
            # Point towards center
            q = self.quaternion_from_euler(0, 0, self.angle_offset + math.pi)
            carrot1_tf.transform.rotation.x = q[0]
            carrot1_tf.transform.rotation.y = q[1]
            carrot1_tf.transform.rotation.z = q[2]
            carrot1_tf.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(carrot1_tf)
        except Exception as e:
            pass
        
        # Broadcast carrot2 (rotating around turtle3)
        try:
            carrot2_tf = TransformStamped()
            carrot2_tf.header.stamp = current_time
            carrot2_tf.header.frame_id = 'turtle3'
            carrot2_tf.child_frame_id = 'carrot2'
            
            # Rotating carrot around turtle3
            radius = 1.5
            carrot2_tf.transform.translation.x = radius * math.cos(self.angle_offset + math.pi)
            carrot2_tf.transform.translation.y = radius * math.sin(self.angle_offset + math.pi)
            carrot2_tf.transform.translation.z = 0.0
            
            # Point towards center
            q = self.quaternion_from_euler(0, 0, self.angle_offset)
            carrot2_tf.transform.rotation.x = q[0]
            carrot2_tf.transform.rotation.y = q[1]
            carrot2_tf.transform.rotation.z = q[2]
            carrot2_tf.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(carrot2_tf)
        except Exception as e:
            pass
        
        # Broadcast static_target
        static_tf = TransformStamped()
        static_tf.header.stamp = current_time
        static_tf.header.frame_id = 'world'
        static_tf.child_frame_id = 'static_target'
        static_tf.transform.translation.x = 8.0
        static_tf.transform.translation.y = 2.0
        static_tf.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, 0)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(static_tf)
        
        # Publish current target name
        target_msg = String()
        target_msg.data = self.current_target
        self.target_pub.publish(target_msg)

    def check_auto_switch(self):
        try:
            # Get distance from turtle2 to current target
            transform = self.tf_buffer.lookup_transform(
                'turtle2', 
                self.current_target, 
                rclpy.time.Time()
            )
            
            distance = math.sqrt(
                transform.transform.translation.x**2 + 
                transform.transform.translation.y**2
            )
            
            if distance < self.switch_threshold:
                self.switch_to_next_target()
                
        except TransformException:
            pass

    def switch_to_next_target(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f'Auto-switched to target: {self.current_target}')

    def switch_target_manual(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f'Manual switch to target: {self.current_target}')

    def keyboard_listener(self):
        while rclpy.ok():
            try:
                # Non-blocking input check
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.readline().strip()
                    if key == 'n':
                        self.switch_target_manual()
            except:
                pass

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q

def main():
    rclpy.init()
    node = TargetSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()