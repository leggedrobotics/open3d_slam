# Open3D Catkin

This package is a catkin wrapper around Open3D.

*Contact:* Julian Nubert (nubertj@ethz.ch)

## Overview

If no Open3D installation is present on the system, *open3d_catkin* is built by compiling the Open3D libraries within the catkin workspace. On a modern desktop computer this takes about 5 minutes. If an Open3D installation exists on the system, this is used instead and wrapped as a catkin package automatically. The latter can be particularly useful if multiple workspaces with this packages are compiled on the computer, or *open3d_catkin* is intended for longer term usage.

## EASY WAY: Install Open3d from a PPA

You can install Open3d from a PPA. The PPA contains Open3d and all dependencies.
First add the PPA to your system:
```
sudo add-apt-repository ppa:roehling/open3d
sudo apt update
sudo apt install libopen3d-dev
```

Proceed to the *open3d_catkin* [compilation step](#compilation).

## EXPERT WAY: Build Open3d from source

<a name="CMake"></a>
### CMake

#### Version >3.18 is required to build open3d_catkin.
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
```sudo ./install_deps.sh```

If you don't want to install open3d locally, skip the next step and proceed to the *open3d_catkin* [compilation step](#compilation).


## Optional Step - Install Open3D Locally
As an optional step, open3d can be installed, either locally in the home folder or as a global system dependency.

Make sure you have installed the correct CMake version (see [above](#CMake))

Clone the following repository:  
```bash
git clone --recursive https://github.com/isl-org/Open3D.git
cd Open3D
git checkout v0.15.1
```

Then, update the submodules:
```bash
git submodule update --init --recursive
```

Create a build directory and build from source:
```bash   
mkdir build
cd build 
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SYSTEM_EIGEN3=OFF -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DCMAKE_INSTALL_PREFIX=${HOME}/Programs/open3d_install ..
make -j$(nproc)
make install
```

### CUDA Support
If you want to compute with cuda support add the following flag when configuring the cmake `-DBUILD_CUDA_MODULE=ON`. In this case you might have to manually specify the path to your nvcc compiler.
This can be done by adding the `CMAKE_CUDA_COMPILER:PATH` flag when invoking the cmake; e.g. `-DCMAKE_CUDA_COMPILER:PATH=/usr/local/cuda-11.5/bin/nvcc`. It can still hapen that you get weird include errors in which case you should not use the system eigen, i.e. `-DUSE_SYSTEM_EIGEN3=OFF` 

### Set Local Paths
To build the catkin package, the simplest way is to add the sufficient environment variable to your bashrc:
```bash
export Open3D_DIR="<your install prefix>/lib/cmake/Open3D"
```
E.g.
```bash 
export Open3D_DIR="$HOME/Programs/open3d_install/lib/cmake/Open3D"
```

<a name="compilation"></a>
## open3d_catkin Compilation
Once Open3D has been installed, compilation of *open3d_catkin* is then really straightforward:
```bash
catkin build open3d_catkin
```
The 3 compilation options are chosen automatically in the following order:
1. open3d is installed locally and the *$Open3D_DIR* environment variable is pointing to the installation location. The success of this is indicated through the message *INFO: Found manually set path to Open3D. Using version located at (some user specified location)*.
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

An example of this can be seen in [../open3d_slam/CMakeLists.txt](https://github.com/leggedrobotics/open3d_slam/blob/dev/ej/open3d_slam/CMakeLists.txt).

### Code Usage
Headers from open3d can then be included as usual:
```cpp
#include <open3d/...>
```
