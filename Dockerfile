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

# clone dependency package repos
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
RUN vcs import src < $IPCAMERA_WS/src/ros2_ipcamera/tools/ros2_dependencies.repos

# build dependency package source
ARG CMAKE_BUILD_TYPE=Release

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# install navigation2 package dependencies
WORKDIR $IPCAMERA_WS
RUN . $ROS_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        $ROS_WS/src \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# source workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$IPCAMERA_WS/install/setup.bash"|'
