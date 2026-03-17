FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG OPEN3D_VERSION=v0.19.0
ARG OPEN3D_BUILD_JOBS=4

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libeigen3-dev \
    libfmt-dev \
    libassimp-dev \
    libglew-dev \
    libglfw3-dev \
    libjpeg-dev \
    libjsoncpp-dev \
    libminizip-dev \
    libmsgpack-dev \
    libnanoflann-dev \
    libopenblas-dev \
    liblapacke-dev \
    libpng-dev \
    cppzmq-dev \
    libqhull-dev \
    libzmq3-dev \
    zlib1g-dev \
    libtbb-dev \
    liblz4-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libxi-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxrandr-dev \
    libwayland-dev \
    libwayland-bin \
    libxkbcommon-dev \
    libudev-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    wayland-protocols \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-interactive-markers \
    ros-jazzy-rosbag2-cpp \
    ros-jazzy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "${OPEN3D_VERSION}" https://github.com/isl-org/Open3D.git /tmp/Open3D && \
    cmake -S /tmp/Open3D -B /tmp/Open3D/build -G Ninja \
      -DCMAKE_CXX_FLAGS=-I/usr/include/minizip \
      -DCMAKE_BUILD_TYPE=Release \
      -DDEVELOPER_BUILD=OFF \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_GUI=OFF \
      -DBUILD_WEBRTC=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_UNIT_TESTS=OFF \
      -DBUILD_BENCHMARKS=OFF \
      -DBUILD_CUDA_MODULE=OFF \
      -DBUILD_PYTHON_MODULE=OFF \
      -DBUILD_AZURE_KINECT=OFF \
      -DBUILD_LIBREALSENSE=OFF \
      -DUSE_SYSTEM_ASSIMP=ON \
      -DUSE_SYSTEM_EIGEN3=ON \
      -DUSE_SYSTEM_GLEW=ON \
      -DUSE_SYSTEM_JPEG=ON \
      -DUSE_SYSTEM_JSONCPP=ON \
      -DUSE_SYSTEM_MSGPACK=ON \
      -DUSE_SYSTEM_PNG=ON \
      -DUSE_SYSTEM_QHULLCPP=ON \
      -DUSE_SYSTEM_ZEROMQ=ON \
      -DUSE_SYSTEM_BLAS=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/Open3D/build -j"${OPEN3D_BUILD_JOBS}" && \
    cmake --install /tmp/Open3D/build && \
    rm -rf /tmp/Open3D && \
    ldconfig

RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-filesystem-dev \
    libboost-iostreams-dev \
    libboost-program-options-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.4-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces/open3d_slam
COPY . /workspaces/open3d_slam

RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --base-paths /workspaces/open3d_slam --packages-up-to open3d_slam_ros"

CMD ["/bin/bash"]
