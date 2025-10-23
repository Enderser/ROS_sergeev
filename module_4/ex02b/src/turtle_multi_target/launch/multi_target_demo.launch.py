import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'switch_threshold',
            default_value='1.0',
            description='Distance threshold for automatic target switching'
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
        
        # Spawn turtle3
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                        '"{x: 8.0, y: 8.0, theta: 0.0, name: \"turtle3\"}"'
                    ],
                    shell=True,
                    output='screen'
                )
            ]
        ),
        
        # Turtle1 tf broadcaster
        Node(
            package='turtle_multi_target',
            executable='turtle1_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            output='screen'
        ),
        
        # Turtle2 tf broadcaster (with delay to ensure turtle2 exists)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle2_tf2_broadcaster',
                    name='turtle2_tf2_broadcaster',
                    output='screen'
                )
            ]
        ),
        
        # Turtle3 tf broadcaster (with delay to ensure turtle3 exists)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle3_tf2_broadcaster',
                    name='turtle3_tf2_broadcaster',
                    output='screen'
                )
            ]
        ),
        
        # Target switcher node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='target_switcher',
                    name='target_switcher',
                    output='screen',
                    parameters=[{'switch_threshold': LaunchConfiguration('switch_threshold')}]
                )
            ]
        ),
        
        # Turtle controller node
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle_controller',
                    name='turtle_controller',
                    output='screen'
                )
            ]
        ),
    ])