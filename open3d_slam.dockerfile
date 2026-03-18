FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV Open3D_DIR=/usr/local/lib/cmake/Open3D

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ninja-build \
    python3-colcon-common-extensions \
    libassimp-dev \
    libeigen3-dev \
    libglfw3-dev \
    libyaml-cpp-dev \
    libsdl2-dev \
    libxi-dev \
    xorg-dev \
    libglu1-mesa-dev \
    python3-dev \
    gfortran \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev \
    autoconf \
    libtool \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-index-cpp \
    ros-jazzy-geometry-msgs \
    ros-jazzy-interactive-markers \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-nav-msgs \
    ros-jazzy-rosbag2-cpp \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v0.15.1 https://github.com/isl-org/Open3D /tmp/Open3D && \
    printf '{}\n' > /tmp/Open3D/examples/test_data/download_file_list.json && \
    sed -i 's/${ExternalProject_CMAKE_ARGS_hidden}/"-DCMAKE_CXX_FLAGS=-fpermissive -Wno-error=changes-meaning"\\n        ${ExternalProject_CMAKE_ARGS_hidden}/' /tmp/Open3D/3rdparty/mkl/tbb.cmake && \
    cmake -S /tmp/Open3D -B /tmp/Open3D/build \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_FLAGS='-Wno-error=maybe-uninitialized -Wno-error=uninitialized' \
      -DCMAKE_CXX_FLAGS='-Wno-error=maybe-uninitialized -Wno-error=uninitialized' \
      -DBUILD_BENCHMARKS=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_GUI=OFF \
      -DBUILD_UNIT_TESTS=OFF \
      -DUSE_SYSTEM_ASSIMP=ON \
      -DUSE_SYSTEM_TBB=OFF \
      -DUSE_SYSTEM_EIGEN3=OFF \
      -DGLIBCXX_USE_CXX11_ABI=ON \
      -DBUILD_PYTHON_MODULE=OFF \
      -DBUILD_WEBRTC=OFF \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/Open3D/build -j"$(nproc)" && \
    cmake --install /tmp/Open3D/build && \
    rm -rf /tmp/Open3D

RUN apt-get update && apt-get install -y --no-install-recommends \
    libglew-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/open3d_slam_ws
COPY . src/open3d_slam
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install --packages-up-to open3d_slam_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash"]
