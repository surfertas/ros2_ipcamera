import os
import pathlib
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with multiple components."""
    ipcamera_node = ComposableNode(
                    package='ros2_ipcamera',
                    node_plugin='ros2_ipcamera::IpCamera',
                    node_name='ipcamera',
                    parameters=[
                        {"rtsp_uri": "rtsp://" },
                        {"image_topic": "image_raw"},
                        {"image_width": 640},
                        {"image_height": 480}
                    ])

    container = ComposableNodeContainer(
            node_name='container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[ipcamera_node],
    )

    return launch.LaunchDescription([container])
