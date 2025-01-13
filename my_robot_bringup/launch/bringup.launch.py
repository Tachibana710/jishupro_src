from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rosbag_command = ['ros2', 'bag', 'record', '--all']

    rosbag_process = ExecuteProcess(
        cmd=rosbag_command,
        output='screen'
    )

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

    state_publisher_node = Node(
        package='state_pkg',
        executable='state_publisher',
        name='state_publisher',
        output='screen'
    )

    control_node = Node(
        package='control_pkg',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    recognition_node = Node(
        package='recognition_pkg',
        executable='recognition_node.py',
        name='recognition_node',
        output='screen'
    )

    camera_calib_node = Node(
        package='recognition_pkg',
        executable='camera_calib.py',
        name='camera_calib_node',
        output='screen'
    )

    return LaunchDescription([
        serial_node,
        foxglove_bridge_launch,
        realsence_launch,
        state_publisher_node,
        control_node,
        recognition_node,
        camera_calib_node,
        # rosbag_process
    ])