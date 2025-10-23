import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

class Turtle2Tf2Broadcaster(Node):
    def __init__(self):
        super().__init__('turtle2_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.turtle2_pose = None
        
        self.subscription = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.handle_turtle_pose,
            10)
        
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def handle_turtle_pose(self, msg):
        self.turtle2_pose = msg

    def broadcast_transform(self):
        if self.turtle2_pose is None:
            return
            
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle2'
        
        t.transform.translation.x = self.turtle2_pose.x
        t.transform.translation.y = self.turtle2_pose.y
        t.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, self.turtle2_pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

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
    node = Turtle2Tf2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()