import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get the package share directory
    pkg_path = get_package_share_directory('diff_drive_gazebo')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    
    # Generate URDF from Xacro
    robot_desc = Command(['xacro ', xacro_file])

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'publish_frequency': 30.0
        }]
    )

    # Static Transform Publisher (world to odom)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # Gazebo Sim
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.5',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for cmd_vel (ROS -> Gazebo)
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ROS-Gazebo Bridge for odometry (Gazebo -> ROS)
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ROS-Gazebo Bridge for joint states (Gazebo -> ROS)
    joint_states_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_states_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz2 Node
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'robot_gazebo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Задержка для спавна робота после запуска Gazebo
    delayed_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_process,
            on_start=[TimerAction(period=3.0, actions=[spawn_entity])],
        )
    )

    # Запуск bridge узлов после спавна робота
    delayed_bridges = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[cmd_vel_bridge, odom_bridge, joint_states_bridge])],
        )
    )

    # Запуск RViz после bridge узлов
    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[rviz_node])],
        )
    )

    return LaunchDescription([
        # Сначала публикуем описание робота и TF
        robot_state_publisher_node,
        static_tf_node,
        joint_state_publisher_gui_node,
        
        # Затем запускаем Gazebo
        gazebo_process,
        
        # И спавним робота с задержкой
        delayed_spawn,
        delayed_bridges,
        delayed_rviz,
    ])