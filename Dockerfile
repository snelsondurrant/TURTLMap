FROM osrf/ros:noetic-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-wstool \
    python3-pip \
    python3-vcstool \
    git \
    cmake \
    build-essential \
    wget \
    unzip \
    # Missing 'package.xml' dependencies
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install gdown

# Download rosbags from Google Drive
WORKDIR /root/bags
RUN gdown --folder 1WLPOTmcomejHVN03Qs8s_scjXoCne35e -O log1
RUN gdown --folder 1WNZGHc0heD0MtYR1I4XDXcoKkpWlZDqO -O log2

WORKDIR /tmp
RUN wget -O gtsam-4.2.zip https://github.com/borglab/gtsam/archive/refs/tags/4.2.zip && \
    unzip gtsam-4.2.zip && \
    cd gtsam-4.2 && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && \
    rm -rf gtsam-4.2 gtsam-4.2.zip

WORKDIR /root/catkin_ws/src
COPY install/turtlmap_ros_https.rosinstall /tmp/turtlmap.rosinstall
RUN wstool init . && \
    wstool merge /tmp/turtlmap.rosinstall && \
    wstool update

# Fix for 'bluerov_visualizer' OpenCV dependency
RUN sed -i 's/project(bluerov_visualizer)/project(bluerov_visualizer)\n\nfind_package(OpenCV REQUIRED)/' bluerov_visualizer/CMakeLists.txt

WORKDIR /root/catkin_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/noetic/setup.sh && catkin build

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
