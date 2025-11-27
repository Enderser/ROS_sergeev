#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SinusoidMovement(Node):
    def __init__(self):
        super().__init__('sinusoid_movement')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Параметры
        self.amplitude = 0.5       # ±50 см влево-вправо
        self.wavelength = 4.0      # длина одной "волны"
        self.speed = 0.6           # м/с
        self.start_time = self.get_clock().now().to_msg().sec

    def timer_callback(self):
        t = (self.get_clock().now().to_msg().sec - self.start_time)
        
        # Позиция по X в идеальной траектории
        x_ideal = (t * self.speed) % self.wavelength
        
        # Фаза для синусоиды
        phase = (2.0 * math.pi * x_ideal) / self.wavelength
        
        # Боковое отклонение (Y)
        y_offset = self.amplitude * math.sin(phase)
        
        # Кривизна = dy/dx ≈ cos(phase) * (2πA/L)
        curvature = (2.0 * math.pi * self.amplitude / self.wavelength) * math.cos(phase)
        angular_z = curvature * self.speed
        
        # Определяем направление движения
        going_forward = (t * self.speed // self.wavelength) % 2 == 0
        
        linear_x = self.speed if going_forward else -self.speed
        # При движении назад
        angular_z = angular_z if going_forward else -angular_z

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

        direction = "Вперёд" if going_forward else "Назад"
        self.get_logger().info(f'{direction} | y={y_offset:+.2f}m | ω={angular_z:+.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SinusoidMovement()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
