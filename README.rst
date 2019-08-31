ROS2 IP Camera Component
====

ROS2 component that publishes raw images taken from an IP camera.

Installation:
----
.. code:: bash

  colcon build --symlink-install
  . install/setup.bash

Usage:
----

Update `rtsp_uri` parameter found in `/launch/ipcamera.launch.py` with the appropriate rtsp uri to your IP camera.

.. code:: bash

  ros2 launch ros2_ipcamera ipcamera.launch.py

Topics:
`/image_raw` - topic for raw image data

References:
----
1. https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
2. http://surfertas.github.io/ros2/2019/08/17/ros2-qos.html
3. https://github.com/klintan/ros2_usb_camera/blob/master/src/usb_camera_driver.cpp
4. https://github.com/ros-perception/image_common/wiki/ROS2-Migration
5. https://github.com/ros2/demos/tree/master/composition
