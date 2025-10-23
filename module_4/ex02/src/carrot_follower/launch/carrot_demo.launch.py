import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'radius',
            default_value='2.0',
            description='Radius of carrot rotation around turtle1'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation',
            default_value='1',
            description='Direction of rotation: 1 for clockwise, -1 for counterclockwise'
        ),
        
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Spawn turtle2
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                '/spawn ',
                'turtlesim/srv/Spawn ',
                '"{x: 5.0, y: 5.0, theta: 0.0, name: \"turtle2\"}"'
            ]],
            shell=True
        ),
        
        # Turtle1 tf broadcaster
        Node(
            package='carrot_follower',
            executable='turtle1_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            output='screen'
        ),
        
        # Turtle2 tf broadcaster
        Node(
            package='carrot_follower',
            executable='turtle2_tf2_broadcaster',
            name='turtle2_tf2_broadcaster',
            output='screen'
        ),
        
        # Carrot frame broadcaster
        Node(
            package='carrot_follower',
            executable='carrot_tf2_broadcaster',
            name='carrot_tf2_broadcaster',
            output='screen',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation')
            }]
        ),
        
        # Turtle2 follower
        Node(
            package='carrot_follower',
            executable='turtle2_carrot_follower',
            name='turtle2_carrot_follower',
            output='screen'
        ),
    ])