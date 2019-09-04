FROM osrf/ros2:nightly

# install ROS2 dependencies
RUN apt-get update && apt-get install -q -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
      wget \
    && rm -rf /var/lib/apt/lists/*

# copy ros package repo
ENV IPCAMERA_WS /opt/ipcamera_ws
RUN mkdir -p $IPCAMERA_WS/src
WORKDIR $IPCAMERA_WS/src
COPY ./ ros2_ipcamera/

# install dependency package dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build dependency package source
ARG CMAKE_BUILD_TYPE=Release

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# source workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$IPCAMERA_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
