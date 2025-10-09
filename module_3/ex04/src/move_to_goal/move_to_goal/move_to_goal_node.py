#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class MoveToGoalNode(Node):

    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta
        
        self.current_pose = None
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Moving turtle to goal: x={goal_x}, y={goal_y}, theta={goal_theta}')

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return
            
        # Calculate distance to goal
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - self.current_pose.theta
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create velocity message
        vel_msg = Twist()
        
        # If we're close to goal position, check orientation
        if distance_to_goal < 0.1:
            # Check if we need to adjust final orientation
            final_angle_diff = self.goal_theta - self.current_pose.theta
            while final_angle_diff > math.pi:
                final_angle_diff -= 2 * math.pi
            while final_angle_diff < -math.pi:
                final_angle_diff += 2 * math.pi
                
            if abs(final_angle_diff) > 0.05:
                vel_msg.angular.z = 0.5 * final_angle_diff
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.get_logger().info('Goal reached!')
        else:
            # Move towards goal
            vel_msg.linear.x = 0.5 * distance_to_goal
            vel_msg.angular.z = 1.0 * angle_diff
            
            # Limit maximum velocities
            vel_msg.linear.x = min(vel_msg.linear.x, 2.0)
            vel_msg.angular.z = max(min(vel_msg.angular.z, 1.0), -1.0)
        
        self.publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal_node <x> <y> <theta>")
        return
    
    try:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2]) 
        goal_theta = float(sys.argv[3])
    except ValueError:
        print("Error: All parameters must be numbers")
        return
    
    node = MoveToGoalNode(goal_x, goal_y, goal_theta)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
