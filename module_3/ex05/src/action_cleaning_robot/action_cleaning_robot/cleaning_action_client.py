#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading
import time

from action_cleaning_robot.action import CleaningTask


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        
        self._action_client = ActionClient(self, CleaningTask, 'CleaningTask')
        
        self.get_logger().info('Cleaning Action Client has been started')
        
        # Запуск последовательности задач после подключения к серверу
        self._send_goals_sequence()

    def _send_goals_sequence(self):
        """Отправка последовательности задач"""
        # Ждем доступности сервера
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Сначала отправляем задачу уборки квадрата 3x3
        self._send_clean_square_goal(3.0)

    def _send_clean_square_goal(self, size):
        """Отправка цели уборки квадрата"""
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = "clean_square"
        goal_msg.area_size = float(size)

        self.get_logger().info(f'Sending clean square goal with size: {size}')

        # Отправляем цель и получаем future для результата
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        # Устанавливаем callback для обработки результата
        future.add_done_callback(lambda future: self._goal_response_callback(future, size))

    def _send_return_home_goal(self):
        """Отправка цели возврата домой"""
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = "return_home"
        goal_msg.target_x = 5.5
        goal_msg.target_y = 5.5

        self.get_logger().info('Sending return home goal to (5.5, 5.5)')

        # Отправляем цель и получаем future для результата
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        # Устанавливаем callback для обработки результата
        future.add_done_callback(self._return_home_goal_response_callback)

    def _goal_response_callback(self, future, size):
        """Callback для обработки ответа на цель уборки квадрата"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                rclpy.shutdown()
                return

            self.get_logger().info('Goal accepted :)')

            # Получаем future для результата
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda future: self._clean_square_result_callback(future, size))
            
        except Exception as e:
            self.get_logger().error(f'Exception in goal response callback: {str(e)}')
            rclpy.shutdown()

    def _return_home_goal_response_callback(self, future):
        """Callback для обработки ответа на цель возврата домой"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Return home goal rejected :(')
                rclpy.shutdown()
                return

            self.get_logger().info('Return home goal accepted :)')

            # Получаем future для результата
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._return_home_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Exception in goal response callback: {str(e)}')
            rclpy.shutdown()

    def _feedback_callback(self, feedback_msg):
        """Callback для обработки feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.progress_percent}%, '
            f'cleaned points: {feedback.current_cleaned_points}, '
            f'position: ({feedback.current_x:.2f}, {feedback.current_y:.2f})'
        )

    def _clean_square_result_callback(self, future, size):
        """Callback для обработки результата уборки квадрата"""
        try:
            result = future.result().result
            self.get_logger().info('Clean square goal completed!')
            self.get_logger().info(
                f'Cleaned points: {result.cleaned_points}, '
                f'Total distance: {result.total_distance:.2f}, '
                f'Success: {result.success}'
            )
            
            if result.success:
                # После успешной уборки отправляем команду возврата домой
                self._send_return_home_goal()
            else:
                self.get_logger().error('Clean square goal failed')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Exception in result callback: {str(e)}')
            rclpy.shutdown()

    def _return_home_result_callback(self, future):
        """Callback для обработки результата возврата домой"""
        try:
            result = future.result().result
            self.get_logger().info('Return home goal completed!')
            self.get_logger().info(
                f'Total cleaned points: {result.cleaned_points}, '
                f'Total distance: {result.total_distance:.2f}, '
                f'Success: {result.success}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Exception in result callback: {str(e)}')
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    cleaning_action_client = CleaningActionClient()
    
    try:
        rclpy.spin(cleaning_action_client)
    except KeyboardInterrupt:
        pass
    finally:
        cleaning_action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()