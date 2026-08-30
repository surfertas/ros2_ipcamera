#!/usr/bin/env python3

# Copyright (c) 2026 Tasuku
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
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessIO
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
    synthetic_stream = ExecuteProcess(
        cmd=[
            sys.executable,
            str(Path(__file__).with_name('synthetic_rtsp.py')),
            '--mediamtx',
            _required_executable('MEDIAMTX_EXECUTABLE', 'mediamtx'),
            '--ffmpeg',
            _required_executable('FFMPEG_EXECUTABLE', 'ffmpeg'),
            '--ffprobe',
            _required_executable('FFPROBE_EXECUTABLE', 'ffprobe'),
            '--config',
            str(Path(__file__).with_name('mediamtx.yml')),
            '--uri',
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
            'frame_id': 'camera_optical_frame',
            'camera_calibration_file': calibration_file,
            'image_width': 640,
            'image_height': 480,
        }],
        output='screen',
    )

    camera_started = {'value': False}

    def start_camera_when_stream_is_ready(event):
        if b'RTSP_READY' not in event.text or camera_started['value']:
            return None
        camera_started['value'] = True
        return [camera, launch_testing.actions.ReadyToTest()]

    return launch.LaunchDescription([
        synthetic_stream,
        RegisterEventHandler(OnProcessIO(
            target_action=synthetic_stream,
            on_stdout=start_camera_when_stream_is_ready,
        )),
    ])


class TestRtspStream(unittest.TestCase):

    def test_publishes_synchronized_frames(self):
        rclpy.init()
        node = rclpy.create_node('rtsp_stream_test')
        images = {}
        camera_infos = {}
        synchronized_stamps = []
        synchronized_stamp_set = set()

        def stamp_key(message):
            return (message.header.stamp.sec, message.header.stamp.nanosec)

        def record_synchronized_pair(stamp):
            if (
                stamp in images
                and stamp in camera_infos
                and stamp not in synchronized_stamp_set
            ):
                synchronized_stamps.append(stamp)
                synchronized_stamp_set.add(stamp)

        def record_image(message):
            stamp = stamp_key(message)
            images.setdefault(stamp, message)
            record_synchronized_pair(stamp)

        def record_camera_info(message):
            stamp = stamp_key(message)
            camera_infos.setdefault(stamp, message)
            record_synchronized_pair(stamp)

        image_subscription = node.create_subscription(
            Image,
            IMAGE_TOPIC,
            record_image,
            qos_profile_sensor_data,
        )
        camera_info_subscription = node.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            record_camera_info,
            qos_profile_sensor_data,
        )

        try:
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(synchronized_stamps) >= 3:
                    break

            self.assertGreaterEqual(
                len(synchronized_stamps),
                3,
                'expected at least three synchronized image/camera-info pairs',
            )

            for stamp in synchronized_stamps[:3]:
                image = images[stamp]
                camera_info = camera_infos[stamp]

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
                self.assertEqual(
                    image.header.frame_id, 'camera_optical_frame'
                )

            self.assertTrue(all(
                later > earlier
                for earlier, later in zip(
                    synchronized_stamps,
                    synchronized_stamps[1:],
                )
            ))
        finally:
            node.destroy_subscription(image_subscription)
            node.destroy_subscription(camera_info_subscription)
            node.destroy_node()
            rclpy.shutdown()
