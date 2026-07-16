# Copyright (c) 2026 BYU FROST Lab
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

# --- Python Scripts Stage ---
FROM python:3.12-slim AS python-scripts

RUN pip install --no-cache-dir rosbags==0.11.3 scipy

# Fetch the dvl_msgs message definitions
RUN apt-get update && apt-get install -y --no-install-recommends git \
  && git clone --depth 1 https://github.com/paagutie/dvl_msgs.git /root/dvl_msgs

COPY ros1_to_ros2_bag.py ned_frd_to_enu_flu.py clean_mocap_ground_truth.py /root/

# --- GTSAM Build Stage ---
FROM osrf/ros:noetic-desktop AS gtsam-builder

ARG DEBIAN_FRONTEND=noninteractive

SHELL [ "/bin/bash", "-c" ]

# Install GTSAM build tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
  apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  ccache

# Build and install GTSAM 4.2 (docs/installation.md)
WORKDIR /tmp/gtsam-build
RUN --mount=type=cache,target=/root/.ccache \
  ccache -z \
  && git clone --depth 1 --branch 4.2 https://github.com/borglab/gtsam.git . \
  && cmake -B build -S . \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_TANGENT_PREINTEGRATION=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache \
  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
  && make -C build -j$(nproc) \
  && make -C build install \
  && ccache -s

# --- Main Stage ---
FROM osrf/ros:noetic-desktop AS turtlmap

ARG DEBIAN_FRONTEND=noninteractive

SHELL [ "/bin/bash", "-c" ]

# Install the GTSAM 4.2 headers and shared libraries (docs/installation.md)
COPY --from=gtsam-builder /usr/local /usr/local
RUN ldconfig

# Install the tools needed to fetch and build the wstool packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
  apt-get update && apt-get install -y --no-install-recommends \
  git \
  python3-pip \
  python3-catkin-tools \
  python3-wstool

# Pull in the required packages (docs/installation.md)
WORKDIR /root/catkin_ws/src
COPY install/turtlmap_ros_https.rosinstall /tmp/turtlmap.rosinstall
RUN wstool init . \
  && wstool merge /tmp/turtlmap.rosinstall \
  && wstool update

# Fix for a missing 'find_package(OpenCV)' in 'bluerov_visualizer'
RUN sed -i 's/project(bluerov_visualizer)/project(bluerov_visualizer)\n\nfind_package(OpenCV REQUIRED)/' bluerov_visualizer/CMakeLists.txt

# Install ROS dependencies via rosdep, plus missing voxblox_ros deps
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
  --mount=type=cache,target=/root/.ros/rosdep \
  apt-get update \
  && rosdep update \
  && rosdep install --from-paths . --ignore-src -r -y \
  && apt-get install -y --no-install-recommends \
  ros-noetic-pcl-ros \
  ros-noetic-cv-bridge

WORKDIR /root/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin build

RUN --mount=type=cache,target=/root/.cache/pip \
  pip3 install scipy

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
  && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
