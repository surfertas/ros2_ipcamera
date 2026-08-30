#!/bin/bash

# Copyright (c) 2026 Tasuku
# SPDX-License-Identifier: Apache-2.0

set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/opt/ipcamera_ws/install/setup.bash"

exec "$@"
