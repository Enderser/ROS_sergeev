from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('diff_drive_bringup'),
                        'launch', 'diff_drive.launch.py')),
        launch_arguments={'rviz': 'true'}.items()
    )

    movement_node = Node(
        package='cool_movement',
        executable='sinusoid_movement',
        name='sinusoid_movement',
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        movement_node
    ])