# Open3D Installation Instructions

## Dependencies
If not done already, the following steps have to be executed.

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

## Build from Source (and install locally)

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

## Set Local Paths
To build the catkin package, the simplest way is to add the sufficient environment variable to your bashrc:
```bash
export Open3D_DIR="<your install prefix>/lib/cmake/Open3D"
```
E.g.
```bash 
export Open3D_DIR="$HOME/Programs/open3d_install/lib/cmake/Open3D"
```

## Python API:

!Download and install anaconda:

Download:
https://www.anaconda.com/products/individual

Install: 
https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html


Inside conda use this command to install the open3d:

conda install -c open3d-admin -c conda-forge open3d

This should install all the required dependencies.