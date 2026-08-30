# Copyright (c) 2026 Tasuku
# SPDX-License-Identifier: Apache-2.0

FROM ros:lyrical-ros-base AS build

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV IPCAMERA_WS=/opt/ipcamera_ws

# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      curl \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${IPCAMERA_WS}
COPY . src/ros2_ipcamera

RUN . "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && rosdep update \
    && rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      -y

ARG CMAKE_BUILD_TYPE=Release
RUN . "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build \
      --symlink-install \
      --cmake-args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"


FROM build AS test

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG MEDIAMTX_VERSION=v1.20.1
RUN architecture="$(dpkg --print-architecture)" \
    && case "${architecture}" in \
      amd64) checksum="81b143f55a5d23d4a8c028d52869c14ea4a59919900528698fcc97a747fd69c6" ;; \
      arm64) checksum="d1689f0bfefb1864e5ed3dcc8495eb2d7ec0a654f90bf3cd48980cb3bd08718a" ;; \
      *) echo "Unsupported architecture: ${architecture}" >&2; exit 1 ;; \
    esac \
    && archive="mediamtx_${MEDIAMTX_VERSION}_linux_${architecture}.tar.gz" \
    && curl --fail --location --output "/tmp/${archive}" \
      "https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/${archive}" \
    && echo "${checksum}  /tmp/${archive}" | sha256sum --check --strict \
    && tar --extract --gzip --file "/tmp/${archive}" --directory /usr/local/bin mediamtx \
    && rm "/tmp/${archive}"

RUN . "${IPCAMERA_WS}/install/setup.bash" \
    && colcon test \
      --packages-select ros2_ipcamera \
      --event-handlers console_direct+ \
    && colcon test-result --verbose


FROM build AS runtime

COPY docker/ros_entrypoint.sh /ros2_ipcamera_entrypoint.sh
RUN chmod +x /ros2_ipcamera_entrypoint.sh

ENTRYPOINT ["/ros2_ipcamera_entrypoint.sh"]
CMD ["bash"]
