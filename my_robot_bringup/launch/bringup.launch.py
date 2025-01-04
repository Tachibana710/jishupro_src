from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    serial_node = Node(
        package='serial_pkg',
        executable='serial_node',
        name='serial_node',
        output='screen'
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        )
    )

    realsence_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_pointcloud': 'true',
            'align_depth': 'true'
        }.items()
    )

    recognition_node = Node(
        package='recognition_pkg',
        executable='recog_node',
        name='recognition_node',
        output='screen'
    )

    return LaunchDescription([
        serial_node,
        foxglove_bridge_launch,
        realsence_launch,
        # recognition_node
    ])