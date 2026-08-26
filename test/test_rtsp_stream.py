#!/usr/bin/env python3

# Copyright (c) 2026 Tasuku Miura
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path
import shutil
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


RTSP_URI = 'rtsp://127.0.0.1:8554/test'
IMAGE_TOPIC = '/ipcamera/image_raw'
CAMERA_INFO_TOPIC = '/ipcamera/camera_info'


def _required_executable(environment_variable, executable_name):
    configured_path = os.environ.get(environment_variable)
    executable_path = configured_path or shutil.which(executable_name)
    if not executable_path:
        raise RuntimeError(
            f'{executable_name} is required for the RTSP integration test; '
            f'install it or set {environment_variable}'
        )
    return executable_path


@pytest.mark.launch_test
def generate_test_description():
    mediamtx = ExecuteProcess(
        cmd=[
            _required_executable('MEDIAMTX_EXECUTABLE', 'mediamtx'),
            str(Path(__file__).with_name('mediamtx.yml')),
        ],
        output='screen',
    )

    synthetic_stream = ExecuteProcess(
        cmd=[
            _required_executable('FFMPEG_EXECUTABLE', 'ffmpeg'),
            '-nostdin',
            '-hide_banner',
            '-loglevel',
            'warning',
            '-re',
            '-f',
            'lavfi',
            '-i',
            'testsrc=size=640x480:rate=10',
            '-an',
            '-c:v',
            'libx264',
            '-preset',
            'ultrafast',
            '-tune',
            'zerolatency',
            '-g',
            '10',
            '-pix_fmt',
            'yuv420p',
            '-f',
            'rtsp',
            '-rtsp_transport',
            'tcp',
            RTSP_URI,
        ],
        output='screen',
    )

    calibration_file = Path(
        get_package_share_directory('ros2_ipcamera'),
        'config',
        'camera_info.yaml',
    ).as_uri()
    camera = launch_ros.actions.Node(
        package='ros2_ipcamera',
        executable='composition',
        name='ipcamera',
        additional_env={
            'OPENCV_FFMPEG_CAPTURE_OPTIONS': 'rtsp_transport;tcp',
        },
        parameters=[{
            'rtsp_uri': RTSP_URI,
            'camera_calibration_file': calibration_file,
            'image_width': 640,
            'image_height': 480,
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        mediamtx,
        TimerAction(period=1.0, actions=[synthetic_stream]),
        TimerAction(period=2.0, actions=[camera]),
        TimerAction(
            period=3.0,
            actions=[launch_testing.actions.ReadyToTest()],
        ),
    ])


class TestRtspStream(unittest.TestCase):

    def test_publishes_synchronized_frames(self):
        rclpy.init()
        node = rclpy.create_node('rtsp_stream_test')
        images = {}
        camera_infos = {}

        def stamp_key(message):
            return (message.header.stamp.sec, message.header.stamp.nanosec)

        image_subscription = node.create_subscription(
            Image,
            IMAGE_TOPIC,
            lambda message: images.setdefault(stamp_key(message), message),
            qos_profile_sensor_data,
        )
        camera_info_subscription = node.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            lambda message: camera_infos.setdefault(
                stamp_key(message), message
            ),
            qos_profile_sensor_data,
        )

        try:
            deadline = time.monotonic() + 20.0
            matching_stamps = []
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                matching_stamps = sorted(images.keys() & camera_infos.keys())
                if len(matching_stamps) >= 2:
                    break

            self.assertGreaterEqual(
                len(matching_stamps),
                2,
                'expected at least two synchronized image/camera-info pairs',
            )

            first_stamp, second_stamp = matching_stamps[:2]
            image = images[first_stamp]
            camera_info = camera_infos[first_stamp]

            self.assertEqual((image.width, image.height), (640, 480))
            self.assertEqual(image.encoding, 'bgr8')
            self.assertEqual(len(image.data), image.step * image.height)
            self.assertEqual(
                (camera_info.width, camera_info.height), (640, 480)
            )
            self.assertEqual(image.header.stamp, camera_info.header.stamp)
            self.assertEqual(
                image.header.frame_id, camera_info.header.frame_id
            )
            self.assertTrue(image.header.frame_id)
            self.assertGreater(second_stamp, first_stamp)
        finally:
            node.destroy_subscription(image_subscription)
            node.destroy_subscription(camera_info_subscription)
            node.destroy_node()
            rclpy.shutdown()
