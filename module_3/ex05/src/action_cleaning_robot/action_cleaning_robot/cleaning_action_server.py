#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math
import threading
from typing import Any

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_cleaning_robot.action import CleaningTask


class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        
        # Action сервер
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'CleaningTask',
            execute_callback=self.execute_callback)
        
        # Издатель для управления черепахой
        self._publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Подписчик для получения позиции
        self._subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        # Текущая позиция черепахи
        self._current_pose = Pose()
        self._current_pose.x = 5.5
        self._current_pose.y = 5.5
        self._current_pose.theta = 0.0
        
        # Переменные для отслеживания прогресса
        self._cleaned_points = 0
        self._total_distance = 0.0
        self._last_x = 0.0
        self._last_y = 0.0
        self._is_moving = False
        
        self.get_logger().info('Cleaning Action Server has been started')

    def pose_callback(self, msg):
        """Callback для получения текущей позиции черепахи"""
        self._current_pose = msg
        
        # Вычисляем пройденное расстояние
        if self._last_x != 0.0 or self._last_y != 0.0:
            distance = math.sqrt((msg.x - self._last_x)**2 + (msg.y - self._last_y)**2)
            self._total_distance += distance
            
            # Считаем "убранные" точки (каждые 0.1 единицы пути)
            if distance > 0.1 and self._is_moving:
                self._cleaned_points += 1
        
        self._last_x = msg.x
        self._last_y = msg.y

    def execute_callback(self, goal_handle):
        """Основной callback для выполнения action"""
        self.get_logger().info(f'Executing goal: {goal_handle.request.task_type}')
        
        # Сброс переменных отслеживания
        self._cleaned_points = 0
        self._total_distance = 0.0
        self._last_x = self._current_pose.x
        self._last_y = self._current_pose.y
        
        goal = goal_handle.request
        
        if goal.task_type == "clean_square":
            return self.execute_clean_square(goal_handle, goal)
        elif goal.task_type == "return_home":
            return self.execute_return_home(goal_handle, goal)
        else:
            self.get_logger().error(f'Unknown task type: {goal.task_type}')
            goal_handle.abort()
            result = CleaningTask.Result()
            result.success = False
            result.cleaned_points = 0
            result.total_distance = 0.0
            return result

    def execute_clean_square(self, goal_handle, goal):
        """Алгоритм уборки по типу принтера - зигзагообразное движение"""
        self._is_moving = True
        
        side_length = goal.area_size
        start_x = self._current_pose.x
        start_y = self._current_pose.y
        
        # Параметры уборки
        line_spacing = 0.25  # Расстояние между линиями (меньше = лучше покрытие)
        speed = 0.5         # Скорость движения
        
        # Вычисляем количество проходов
        num_passes = max(2, int(side_length / line_spacing))
        current_pass = 0
        
        feedback_msg = CleaningTask.Feedback()
        
        self.get_logger().info(f'Starting printer-style cleaning: {side_length}x{side_length}m, {num_passes} passes')
        
        try:
            # Начинаем с левого нижнего угла
            # Сначала двигаемся к начальной точке (левый нижний угол)
            self._move_to_position(start_x - side_length/2, start_y - side_length/2, goal_handle, feedback_msg)
            
            # Основной цикл уборки - зигзагообразное движение
            for pass_num in range(num_passes):
                if not rclpy.ok() or goal_handle.is_cancel_requested:
                    break
                
                current_pass = pass_num
                is_even_pass = (pass_num % 2 == 0)
                
                # Определяем целевую точку для этого прохода
                if is_even_pass:
                    # Четные проходы: двигаемся вправо
                    target_x = start_x + side_length/2
                else:
                    # Нечетные проходы: двигаемся влево  
                    target_x = start_x - side_length/2
                
                # Текущая Y координата для этого прохода
                current_y = start_y - side_length/2 + (pass_num * line_spacing)
                
                self.get_logger().info(f'Pass {pass_num + 1}/{num_passes}: moving to Y={current_y:.2f}')
                
                # Поворачиваем в нужном направлении
                if is_even_pass:
                    self._turn_to_angle(0, goal_handle)  # Вправо (0 градусов)
                else:
                    self._turn_to_angle(math.pi, goal_handle)  # Влево (180 градусов)
                
                # Движение вдоль прохода
                self._move_to_position(target_x, current_y, goal_handle, feedback_msg)
                
                # Если не последний проход, перемещаемся к следующей линии
                if pass_num < num_passes - 1:
                    # Поворачиваем для движения к следующей линии
                    self._turn_to_angle(math.pi/2, goal_handle)  # Вверх (90 градусов)
                    
                    # Короткое движение к следующей линии
                    next_y = current_y + line_spacing
                    self._move_to_position(self._current_pose.x, next_y, goal_handle, feedback_msg)
                
                # Обновление прогресса
                progress = int(((pass_num + 1) * 100) / num_passes)
                feedback_msg.progress_percent = min(99, progress)  # Оставляем 1% для финализации
                feedback_msg.current_cleaned_points = self._cleaned_points
                feedback_msg.current_x = self._current_pose.x
                feedback_msg.current_y = self._current_pose.y
                goal_handle.publish_feedback(feedback_msg)
            
            # Финальный прогресс
            feedback_msg.progress_percent = 100
            goal_handle.publish_feedback(feedback_msg)
            
            # Остановка
            stop_msg = Twist()
            self._publisher.publish(stop_msg)
            self._is_moving = False
            
            # Результат
            result = CleaningTask.Result()
            result.success = True
            result.cleaned_points = self._cleaned_points
            result.total_distance = self._total_distance
            
            goal_handle.succeed()
            self.get_logger().info(f'Printer-style cleaning completed! Cleaned {self._cleaned_points} points')
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error during cleaning: {str(e)}')
            stop_msg = Twist()
            self._publisher.publish(stop_msg)
            self._is_moving = False
            
            result = CleaningTask.Result()
            result.success = False
            result.cleaned_points = self._cleaned_points
            result.total_distance = self._total_distance
            return result

    def _move_to_position(self, target_x, target_y, goal_handle, feedback_msg):
        """Движение к целевой позиции с постоянной скоростью"""
        tolerance = 0.1
        max_speed = 0.5
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                raise Exception("Goal canceled")
            
            # Вычисляем расстояние и направление
            dx = target_x - self._current_pose.x
            dy = target_y - self._current_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Проверяем достижение цели
            if distance < tolerance:
                break
            
            # Вычисляем угол к цели
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self._current_pose.theta
            
            # Нормализация угла
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Создаем команду движения
            msg = Twist()
            
            # Корректировка угла если нужно
            if abs(angle_diff) > 0.2:  # ~11.5 градусов
                msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Движение к цели
                msg.linear.x = min(max_speed, distance * 0.5)  # Замедляемся при приближении
            
            self._publisher.publish(msg)
            
            # Частота обновления
            rclpy.spin_once(self, timeout_sec=0.1)

    def _turn_to_angle(self, target_angle, goal_handle):
        """Поворот к заданному углу"""
        tolerance = 0.05  # ~3 градуса
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                raise Exception("Goal canceled")
            
            angle_diff = target_angle - self._current_pose.theta
            
            # Нормализация угла
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if abs(angle_diff) < tolerance:
                break
            
            msg = Twist()
            msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            self._publisher.publish(msg)
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Остановка после поворота
        stop_msg = Twist()
        self._publisher.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.1)

    def execute_return_home(self, goal_handle, goal):
        """Выполнение возврата в домашнюю позицию"""
        self._is_moving = True
        
        target_x = goal.target_x
        target_y = goal.target_y
        tolerance = 0.1
        
        feedback_msg = CleaningTask.Feedback()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._is_moving = False
                result = CleaningTask.Result()
                result.success = False
                result.cleaned_points = self._cleaned_points
                result.total_distance = self._total_distance
                return result
            
            # Вычисление направления к цели
            dx = target_x - self._current_pose.x
            dy = target_y - self._current_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Проверка достижения цели
            if distance < tolerance:
                stop_msg = Twist()
                self._publisher.publish(stop_msg)
                self._is_moving = False
                
                result = CleaningTask.Result()
                result.success = True
                result.cleaned_points = self._cleaned_points
                result.total_distance = self._total_distance
                
                goal_handle.succeed()
                self.get_logger().info('Return home completed successfully')
                return result
            
            # Вычисление угла к цели
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self._current_pose.theta
            
            # Нормализация угла
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            msg = Twist()
            
            # Поворот к цели
            if abs(angle_diff) > 0.1:
                msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Движение к цели
                msg.linear.x = min(1.0, distance)
            
            self._publisher.publish(msg)
            
            # Обновление feedback
            max_distance = math.sqrt((5.5 - self._current_pose.x)**2 + (5.5 - self._current_pose.y)**2)
            progress = 100.0 * (1.0 - distance/max_distance) if max_distance > 0 else 100.0
            feedback_msg.progress_percent = int(progress)
            feedback_msg.current_cleaned_points = self._cleaned_points
            feedback_msg.current_x = self._current_pose.x
            feedback_msg.current_y = self._current_pose.y
            goal_handle.publish_feedback(feedback_msg)
            
            # Задержка для плавного движения
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self._is_moving = False
        result = CleaningTask.Result()
        result.success = False
        return result


def main(args=None):
    rclpy.init(args=args)
    
    cleaning_action_server = CleaningActionServer()
    
    try:
        rclpy.spin(cleaning_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        cleaning_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()