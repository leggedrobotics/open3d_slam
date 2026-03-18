# Open3D Colcon

This package wraps an installed Open3D for `colcon` workspaces based on `ament_cmake`.

*Contact:* Julian Nubert (nubertj@ethz.ch)

## Overview

`open3d_colcon` expects Open3D to be available through CMake, typically via `Open3D_DIR` or a system install in `/usr/local`.

The repository Dockerfile handles the Open3D source build for the supported Jazzy flow. Outside Docker, install Open3D yourself and point this package at it if needed.

## Open3D Prerequisites

If you want to build Open3D from source yourself, install the
Open3D build dependencies first:

```bash
cd open3d_colcon
sudo ./install_deps.sh assume-yes
```

If you already have a local Open3D installation, point the package at it:

```bash
export Open3D_DIR="$HOME/.local/lib/cmake/Open3D"
```

## Build with Colcon

Create a workspace and clone this repository into `src`:

```bash
mkdir -p ~/open3d_slam_ws/src
cd ~/open3d_slam_ws/src
git clone https://github.com/leggedrobotics/open3d_slam.git
cd ..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to open3d_colcon --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To build the supported Jazzy core instead of only the wrapper package:

```bash
colcon build --symlink-install --packages-up-to open3d_slam_yaml_io --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Use `open3d_colcon` in Another Package

Downstream `ament_cmake` packages should depend on `open3d_colcon`.

### CMakeLists.txt
```cmake
find_package(ament_cmake REQUIRED)
find_package(open3d_colcon REQUIRED)

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node open3d_colcon)

ament_package()
```

### Code Usage
Headers from Open3D can then be included as usual:

```cpp
#include <open3d/...>
```
