from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # Ignition GazeboとROS2ブリッジ
    ros_ign_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/world/panda_world/pose/info@ignition.msgs.Pose_V@ros2/ignition_msgs/msg/Pose_V'
        ],
        output='screen'
    )

    return LaunchDescription([
        ros_ign_bridge
    ])
