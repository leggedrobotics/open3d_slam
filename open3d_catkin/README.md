# Open3D Catkin

This package is a catkin wrapper around Open3D.

*Contact:* Julian Nubert (nubertj@ethz.ch)

## Overview

If no open3d installation is present on the system, _open3d_catkin_ is built by compiling the open3d libraries within the catkin workspace. On a modern desktop computer this takes about 5 minutes. If an Open3D installation exists on the system, this is used instead and wrapped as a catkin package automatically. The latter can be particularly useful if multiple workspaces with this packages are compiled on the computer, or _open3d_catkin_ is intended for longer term usage.

## Mandatory Steps

### Cmake
Version >3.18 is required to build open3d_catkin.
For installation, do:
* Download the latest tar archive from https://cmake.org/download/
* ```tar -xf cmake-<version>-rc4.tar.gz```, where ```<version>``` can for example be _3.23.1_.
* Then install this cmake version by doing
```
cd cmake-<version>-rc4.tar.gz
./configure
make -j12
sudo make install
```

### Open3D Dependencies
Execute either the original installation script from open3d: 
[script](https://github.com/isl-org/Open3D/blob/v0.13.0/util/install_deps_ubuntu.sh),
or our fetched version via
```sudo ./install_Deps.sh```

## Optional Step
As an optional step, open3d can be installed, either locally in the home folder or as a global system dependency.
For the Open3D installation instructions refer to this [file](./install_open3d.md).

## Compilation
Compilation of _open3d_catkin_ is then really straightforward:
```bash
catkin build open3d_catkin
```
The 3 compilation options are chosen automatically in the following order:
1. open3d is installed locally and the _$Open3D_DIR_ environment variable is pointing to the installation location. The success of this is indicated through the message _INFO: Found manually set path to Open3D. Using version located at <location>._.
_
2. open3d is installed globally.
Both of these options should compile within a few seconds.
3. If none of the before cases holds, open3d is automatically pulled locally and compiled inside the workspace.  on an Intel i9 12900K this takes roughly _4min 30s_.

## Usage of open3d_catkin in your project
Usage in your catkin project is then straightforward.

### CMakeLists.txt
```cmake
set(CATKIN_PACKAGE_DEPENDENCIES
  open3d_catkin
)
...
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)
...

catkin_package(
  ...
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)
...
include_directories(
  ${catkin_INCLUDE_DIRS}
  ...
)
...
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ...
)

```

An example of this can be seen in [../open3d_slam/CMakeLists.txt](../open3d_slam/CMakeLists.txt).

### Code Usage
Headers from open3d can then be included as usual:
```cpp
#include <open3d/...>
```