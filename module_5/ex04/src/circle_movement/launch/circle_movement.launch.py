import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('diff_drive_bringup')
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'diff_drive.launch.py')),
        launch_arguments={'rviz': 'true'}.items()  # Включаем RViz
    )

    # Узел для публикации в /cmd_vel
    circle_node = Node(
        package='circle_movement',
        executable='circle_movement',
        name='circle_movement',
        output='screen'
    )

    return LaunchDescription([
        included_launch,
        circle_node
    ])
