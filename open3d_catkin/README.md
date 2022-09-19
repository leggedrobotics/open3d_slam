# Open3D Catkin

This package is a catkin wrapper around Open3D.

*Contact:* Julian Nubert (nubertj@ethz.ch)

## 1 - Overview

If no Open3D installation is present on the system, *open3d_catkin* is built by compiling the Open3D libraries within the catkin workspace. On a modern desktop computer this takes about 5 minutes. 
If an Open3D installation exists on the system, this is used instead and wrapped as a catkin package automatically. The latter can be particularly useful if multiple workspaces with these packages are compiled on the computer, or *open3d_catkin* is intended for longer-term usage. There are two ways to install Open3D:
1. From a PPA ([Section 2](#ppa)) or

By compiling it locally or globally from source ([Section 3](#from_source)). This can be done by executing [Step 3.0](#requirements) **and** either 

2. explicitly as shown in [Section 3.1](#explicit_compilation), or
3. automatically compiled in the catkin workspace as shown in [Section 3.2](#automatic_compilation).

<a name="ppa"></a>
## 2 - Install Open3d from a PPA

**Execute EITHER this step OR [Section 3](#from_source) for compiling Open3D from source.**

You can install Open3d from a PPA. The PPA contains Open3d and all dependencies needed to run *open3d_slam*, but if additional functionality or optimized performance is required [Section 3](#from_source) might be more suitable.
First add the PPA to your system:
```bash
sudo add-apt-repository ppa:roehling/open3d
sudo apt update
```
Then install the library
```bash
sudo apt install libopen3d-dev
```

Next, proceed to the *open3d_catkin* compilation step in [Section 4](#catkin_compilation).

<a name="from_source"></a>
## 3 - Build Open3d from source

**This is only needed if Open3D was not installed in [Section 2](#ppa).**

Building Open3D from source can lead to better performance, removes the need for the required PPA and can potentially be build with CUDA support. On an Intel i9 12900K this roughly takes 4-5 minutes.

<a name="requirements"></a>
### 3.0 - Requirements
The requirements in Section 3.0 have to be installed for any open-source installation.

<a name="CMake"></a>
#### 3.0.1 - CMake

For compiling Open3D from source, **CMake version >3.18** is required.
For installation, do:
1. Download the latest **source distribution** tar archive from [https://cmake.org/download/](https://cmake.org/download/) for your operating system (e.g. *cmake-3.24.1.tar.gz*).
2. Extract it by doing ```tar -xf cmake-<version>.tar.gz```, where ```<version>``` can for example be _3.24.1_.
3. Then install this cmake version by doing
```bash
cd cmake-<version>-rc4.tar.gz
./configure
make -j$(nproc)
sudo make install
```

#### 3.0.2 - Additional Dependencies
Execute either the original installation script from open3d:
* [script](https://github.com/isl-org/Open3D/blob/v0.15.1/util/install_deps_ubuntu.sh)

OR
* run our [fetched installation script](https://github.com/leggedrobotics/open3d_slam/blob/master/open3d_catkin/install_deps.sh) (they are the same) via
```bash
sudo ./install_deps.sh
```

#### 3.0.3 - CUDA Support (Optional)
**NOTE: this is not needed for open3d_slam, but might be helpful if you use open3d_catkin in other projects.**

If you want to compute with cuda support add the following flag when configuring the cmake `-DBUILD_CUDA_MODULE=ON`. In this case you might have to manually specify the path to your nvcc compiler.
This can be done by adding the `CMAKE_CUDA_COMPILER:PATH` flag when invoking the cmake; e.g. `-DCMAKE_CUDA_COMPILER:PATH=/usr/local/cuda-11.5/bin/nvcc`. It can still happen that you get weird include errors in which case you should not use the system eigen, i.e. `-DUSE_SYSTEM_EIGEN3=OFF` 


<a name="explicit_compilation"></a>
### 3.1 - Explicit Compilation of Open3D

**NOTE: This step is optional. If no explicit compilation is performed, Open3D is automatically compiled in the workspace, see [Section 3.2](#automatic_compilation).**

For the explicit compilation, e.g. to share it among workspaces, first clone the following repository and checkout the correct branch:  
```bash
git clone --recursive https://github.com/isl-org/Open3D.git
cd Open3D
git checkout v0.15.1
```

Then, update the submodules:
```bash
git submodule update --init --recursive
```
Since we will use system install of the fmt libraries, run the following
```bash
sudo apt install libfmt-dev
```

Create a build directory and build from source:
```bash   
mkdir build
cd build 
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SYSTEM_EIGEN3=OFF -DUSE_SYSTEM_FMT=ON -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DCMAKE_INSTALL_PREFIX:PATH=${HOME}/.local .. # If global install is desired, remove the -DCMAKE_INSTALL_PREFIX-var
make -j$(nproc)
make install # if global installation, add "sudo"
```
This will move all *open3d* includes and libraries to the correct location. 

### Set Local Paths
If open3d has been installed locally, the simplest way to build the catkin package is to add the sufficient environment variable to your bashrc, i.e. either
```bash
export Open3D_DIR="<your install prefix>/lib/cmake/Open3D"
```
e.g.
```bash 
export Open3D_DIR="$HOME/.local/lib/cmake/Open3D"
```
OR
```bash
export CMAKE_PREFIX_PATH=$HOME/.local/:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib/:$LD_LIBRARY_PATH
export LIBRARY_PATH=${LIBRARY_PATH}:${LD_LIBRARY_PATH}
```

Next, proceed to the *open3d_catkin* compilation step in [Section 4](#catkin_compilation).

<a name="automatic_compilation"></a>
### 3.2 - Automatic Compilation of Open3D
For this part, no additional steps are required, just proceed to [Section 4](#compilation).

Next, proceed to the *open3d_catkin* [compilation step](#catkin_compilation).

<a name="catkin_compilation"></a>
## 4 - open3d_catkin Compilation
Once Open3D has been installed, compilation of *open3d_catkin* is then really straightforward.
Make sure you have a catkin workspace with this repository:
```bash
git clone https://github.com/leggedrobotics/open3d_slam.git
```
Then you can build the package in release mode.
```bash
catkin build open3d_catkin -DCMAKE_BUILD_TYPE=Release
```
As introduced before, the 3 compilation options are chosen automatically in the following order:
1. open3d is installed locally and the *$Open3D_DIR* environment variable is pointing to the installation location. The success of this is indicated through the message *INFO: Found manually set path to Open3D. Using version located at (some user-specified location)*.
2. open3d is installed globally.
Both of these options should compile within a few seconds.
3. If none of the before cases holds, open3d is automatically pulled locally and compiled inside the workspace.

The actual compilation of *open3d_slam* and *open3d_slam_ros* should require less than a minute.

## 5 - Usage of open3d_catkin in your project
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
