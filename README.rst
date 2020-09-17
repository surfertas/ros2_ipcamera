ROS2 IP Camera Component
====

ROS2 component that publishes raw images taken from an IP camera.

Note: to cross-compile for ARM architecture see the related article_.

.. _article: http://surfertas.github.io/ros2/cross-compile/2019/10/14/crosscompile.html

Installation:
----

.. code:: bash

  git clone https://github.com/surfertas/ros2_ipcamera.git
  colcon build --symlink-install
  . install/setup.bash

Docker:

.. code:: bash

  git clone https://github.com/surfertas/ros2_ipcamera.git
  cd ros2_ipcamera
  sudo docker build -t ros2_ipcamera/latest .

Usage:
----

1. Update ``rtsp_uri`` parameter found in ``/config/ipcamera.yaml`` with the appropriate rtsp uri to your IP camera.
2. Set the width and height to match the resolution of the IP camera. The node does not resize the image, but only sets the capture.
3. Generate a camera_info.yaml file and place in ``/config``.

.. code:: bash

  ros2 run ros2_ipcamera composition

  # Alternatively use the launch file
  ros2 launch ros2_ipcamera ipcamera.launch.py

Docker:

.. code:: bash

  # Update rtsp_uri in the yaml file.
  sudo docker run -it ros2_ipcamera/latest bash
  vi src/ros2_ipcamera/config/ipcamera.yaml
  source ./install/setup.bash
  ros2 launch ros2_ipcamera ipcamera.launch.py

Topics:

``/ipcamera/image_raw`` - topic for raw image data

``/ipcamera/camera_info`` - topic for camera info

References:
----
1. https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
2. http://surfertas.github.io/ros2/2019/08/17/ros2-qos.html
3. https://github.com/klintan/ros2_usb_camera/blob/master/src/usb_camera_driver.cpp
4. https://github.com/ros-perception/image_common/wiki/ROS2-Migration
5. https://github.com/ros2/demos/tree/master/composition
6. https://github.com/christianrauch/raspicam2_node/blob/master/src/RasPiCamPublisherNode.cpp
