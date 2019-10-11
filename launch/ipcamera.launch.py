import os
import pathlib
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with multiple components."""

    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('ros2_ipcamera'), 'config')
    # Alternatively can use package:// per:
    # https://answers.ros.org/question/333521/ros2-url-to-camera_info-yaml-not-being-recognized/
    config_file = 'file://' + os.path.join(config_dir, "camera_info.yaml")

    ipcamera_node = ComposableNode(
                    package='ros2_ipcamera',
                    node_plugin='ros2_ipcamera::IpCamera',
                    node_name='ipcamera',
                    parameters=[
                        {"rtsp_uri": "rtsp://"},
                        {"image_topic": "image_raw"},
                        {"camera_calibration_file": config_file},
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
