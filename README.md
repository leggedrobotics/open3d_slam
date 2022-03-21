# Open3D SLAM

This package contains the code for a open3D SLAM framework.

DISCLAIMER  
This package has been devloped as a hobby project and it is under heavy development. Hence we cannot (yet) guarantee any stability.

## Dependencies

This package depends on Open3D. There are currently two ways of compiling open3d_slam:
1.  By installing Open3D beforehand, see below for the instructions.
2.  By compiling Open3D alongside open3d_slam in the catkin workspace.

Both versions are possible, and the compilation is automatically handled by open3d_catkin.

### open3d_catkin
The options for compiling are taken in the following order:
1.  If an environment variable _${Open3D_DIR}_ is set, the Open3D installation at this location is chosen.
2.  If 1. is not given, but Open3D is installed (i.e. can be found by cmake), this installation is taken.
3.  If the two points before are not given, Open3D will automatically be pulled from GitHub and compiled alongside the other packages in the workspace.

Please note that the latter option is more flexible but slower, as open3d is first compiled from source.

### Docker
We provide a docker image with pre-compiled Open3D binaries.

#### Pulling the Image from Docker Hub

The image can be pulled from docker hub using
```bash
docker pull rslethz/m545-gpu:open3d
```

#### Running the Docker Image

For running the docker image we recommend the usage of the run.sh script from this [repository](https://github.com/leggedrobotics/m545_docker).

After building the image (or pulling it from docker hub), this can be done by typing
```bash
./bin/run.sh --type=gpu --tag=open3d
```


## Compiling the Repo

If Open3D is installed (e.g. inside the docker), the compilation can be performed using 
```bash
catkin build open3d_slam
```
If not instllation can be found, also Open3D will be compiled from source automatically.

## Running the Repo

Add the required config to the launch file `mapping.launch`.
Then after sourcing the workspace do
```bash
roslaunch open3d_slam mapping.launch
```

----------------------------------------------

## Open3D Installation instructions

### C++ API:

#### CMake

You might need to update your cmake, since the open3d requires cmake >= 3.18

See the instructions here:
https://apt.kitware.com/

If the instructions below fail, refer to t he installation manual:
http://www.open3d.org/docs/release/compilation.html

#### Additional dependenceis
Install additional dependencies by running
```bash
sudo ./open3d_catkin/install_deps.sh
```

#### Build from Source

Clone the following repository:  

```bash
git clone --recursive https://github.com/isl-org/Open3D.git  
git checkout v0.15.1
```

Update the submodules:
```bash
git submodule update --init --recursive
```

Create a build directory and build from source:
```bash   
mkdir build
cd build 
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SYSTEM_EIGEN3=OFF -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DCMAKE_INSTALL_PREFIX=${HOME}/Programs/open3d_install ..
make -j8
make install
```

If you want to compute with cuda support add the following flag when configuring the cmake `-DBUILD_CUDA_MODULE=ON`. In this case you might have to manually specify the path to your nvcc compiler.
This can be done by adding the `CMAKE_CUDA_COMPILER:PATH` flag when invoking the cmake; e.g. `-DCMAKE_CUDA_COMPILER:PATH=/usr/local/cuda-11.5/bin/nvcc`. It can still hapen that you get weird include errors in which case you should not use the system eigen, i.e. `-DUSE_SYSTEM_EIGEN3=OFF` 

#### Set Paths
To build the catkin package, the simplest way is to add the sufficient environment variable to your bashrc:
```bash
export Open3D_DIR="${your install prefix}/lib/cmake/Open3D"
```
E.g.
```bash 
export Open3D_DIR="$HOME/Programs/open3d_install/lib/cmake/Open3D"
```

### Python API:

!Download and install anaconda:

Download:
https://www.anaconda.com/products/individual

Install: 
https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html


Inside conda use this command to install the open3d:

conda install -c open3d-admin -c conda-forge open3d

This should install all the required dependencies.
