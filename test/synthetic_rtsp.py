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

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mediamtx', required=True)
    parser.add_argument('--ffmpeg', required=True)
    parser.add_argument('--ffprobe', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--uri', required=True)
    return parser.parse_args()


def wait_for_port(process, host, port, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                'MediaMTX exited before its RTSP port was ready'
            )
        try:
            with socket.create_connection((host, port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError('MediaMTX RTSP port did not become ready')


def wait_for_stream(process, ffprobe, uri, timeout):
    command = [
        ffprobe,
        '-v',
        'error',
        '-rtsp_transport',
        'tcp',
        '-select_streams',
        'v:0',
        '-show_entries',
        'stream=width,height',
        '-of',
        'csv=p=0:s=x',
        uri,
    ]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                'FFmpeg exited before the RTSP stream was ready'
            )
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            continue
        if result.returncode == 0 and result.stdout.strip() == '640x480':
            return
        time.sleep(0.1)
    raise TimeoutError('synthetic RTSP stream did not become ready')


def terminate(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3.0)


def main():
    arguments = parse_arguments()
    parsed_uri = urlparse(arguments.uri)
    stop_requested = False
    mediamtx = None
    ffmpeg = None

    def request_stop(_signal_number, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        mediamtx = subprocess.Popen(
            [arguments.mediamtx, arguments.config],
            start_new_session=True,
        )
        wait_for_port(
            mediamtx,
            parsed_uri.hostname or '127.0.0.1',
            parsed_uri.port or 554,
            timeout=10.0,
        )

        ffmpeg = subprocess.Popen([
            arguments.ffmpeg,
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
            arguments.uri,
        ], start_new_session=True)
        wait_for_stream(ffmpeg, arguments.ffprobe, arguments.uri, timeout=20.0)
        print('RTSP_READY', flush=True)

        while not stop_requested:
            if mediamtx.poll() is not None or ffmpeg.poll() is not None:
                raise RuntimeError(
                    'synthetic RTSP fixture exited unexpectedly'
                )
            time.sleep(0.1)
        return 0
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f'synthetic RTSP fixture failed: {error}', file=sys.stderr)
        return 1
    finally:
        terminate(ffmpeg)
        terminate(mediamtx)


if __name__ == '__main__':
    sys.exit(main())
