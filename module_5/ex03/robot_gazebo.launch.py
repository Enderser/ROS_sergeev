import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get the package share directory
    pkg_path = get_package_share_directory('diff_drive_gazebo')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
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

    # Joint State Publisher GUI для ручного управления суставами
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
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
            '-z', '0.5'
        ],
        output='screen'
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'{world_file} -r'
        }.items()
    )

    # ROS-GZ Bridge for cmd_vel
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static Transform Publisher (world to base_link) - ОСНОВНОЙ фрейм
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        static_tf_node,
        spawn_entity,
        cmd_vel_bridge,
        rviz_node,
    ])