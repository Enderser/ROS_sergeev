import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='5.0',
            description='Time delay in seconds for turtle2 to follow turtle1'
        ),
        
        # Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Spawn turtle2
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                        '"{x: 2.0, y: 2.0, theta: 0.0, name: \"turtle2\"}"'
                    ],
                    shell=True,
                    output='screen'
                )
            ]
        ),
        
        # Turtle1 tf broadcaster (publishes current pose)
        Node(
            package='time_delayed_turtle',
            executable='turtle1_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            output='screen'
        ),
        
        # Turtle2 tf broadcaster
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='time_delayed_turtle',
                    executable='turtle2_tf2_broadcaster',
                    name='turtle2_tf2_broadcaster',
                    output='screen'
                )
            ]
        ),
        
        # Delayed tf broadcaster (publishes turtle1 pose from the past)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='time_delayed_turtle',
                    executable='delayed_tf2_broadcaster',
                    name='delayed_tf2_broadcaster',
                    output='screen',
                    parameters=[{'delay': LaunchConfiguration('delay')}]
                )
            ]
        ),
        
        # Turtle2 follower (follows the delayed pose)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='time_delayed_turtle',
                    executable='turtle2_time_follower',
                    name='turtle2_time_follower',
                    output='screen'
                )
            ]
        ),
    ])