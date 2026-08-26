ROS 2 IP Camera Component
=========================

A ROS 2 component that reads an RTSP stream through OpenCV and publishes raw
images together with camera calibration information.

The current target is ROS 2 Lyrical Luth on Ubuntu 26.04.

Installation
------------

Clone this package into a ROS 2 workspace, install its dependencies, and build
it:

.. code:: bash

  mkdir -p ~/ipcamera_ws/src
  cd ~/ipcamera_ws/src
  git clone https://github.com/surfertas/ros2_ipcamera.git
  cd ..
  rosdep install --from-paths src --ignore-src --rosdistro lyrical -y
  colcon build --symlink-install
  source install/setup.bash

Usage
-----

Set ``rtsp_uri``, ``image_width``, and ``image_height`` in
``config/ipcamera.yaml``. The configured dimensions must match a resolution
provided by the camera; this node does not resize images. Replace
``config/camera_info.yaml`` with the calibration for the camera.

Then start either the standalone executable or the composable-node launch file:

.. code:: bash

  ros2 run ros2_ipcamera composition

  # Alternatively:
  ros2 launch ros2_ipcamera ipcamera.launch.py

The node publishes:

* ``/ipcamera/image_raw`` — raw image data
* ``/ipcamera/camera_info`` — matching calibration and timestamps

Docker
------

The default Docker target builds a ROS 2 Lyrical runtime image:

.. code:: bash

  docker build --tag ros2_ipcamera:lyrical .

Pass camera parameters directly when starting the standalone node:

.. code:: bash

  docker run --rm --network host ros2_ipcamera:lyrical \
    ros2 run ros2_ipcamera composition --ros-args \
      -p rtsp_uri:=rtsp://camera-host/stream \
      -p image_width:=640 \
      -p image_height:=480 \
      -p camera_calibration_file:=file:///opt/ipcamera_ws/src/ros2_ipcamera/config/camera_info.yaml

Testing without a camera
------------------------

The integration test starts MediaMTX, publishes FFmpeg's deterministic test
pattern over RTSP, launches the node, and verifies through its public ROS topics
that:

* at least two synchronized image and camera-info pairs arrive;
* images are 640 x 480 ``bgr8`` frames with a complete data buffer;
* camera calibration dimensions match the images; and
* timestamps advance.

Run the complete build and test in Docker:

.. code:: bash

  docker build --target test --tag ros2_ipcamera:test .

To run it in an existing ROS 2 Lyrical workspace, install ``ffmpeg`` and put the
``mediamtx`` executable on ``PATH``, then run:

.. code:: bash

  colcon build --symlink-install
  source install/setup.bash
  colcon test --packages-select ros2_ipcamera --event-handlers console_direct+
  colcon test-result --verbose

GitHub Actions runs the same Docker test target on every push and pull request.

References
----------

1. https://docs.ros.org/en/lyrical/
2. https://github.com/bluenviron/mediamtx
3. https://github.com/ros-perception/image_common
