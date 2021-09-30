
## Installation instructions


### For the python API:

!Download and install anaconda:

Download:
https://www.anaconda.com/products/individual

Install: 
https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html


Inside conda use this command to install the open3d:

conda install -c open3d-admin -c conda-forge open3d

This should install all the required dependencies.


### For the cpp API:

You might need to update your cmake, since the open3d requires cmake >= 3.18

See the instructions here:
https://apt.kitware.com/

Install the open3d from source:
http://www.open3d.org/docs/release/compilation.html

`git clone --recursive https://github.com/intel-isl/Open3D`

Make sure you checkout this tag v0.13.0

`git checkout v0.13.0`

Update the submodules:
`git submodule update --init --recursive`

Make sure that you install the dependencies.

http://www.open3d.org/docs/release/compilation.html#install-dependencies

The script does not install the following deps which you have to install manually:
`sudo apt install libfmt-dev`
`sudo apt install libglfw3-dev`

Create a build directory:
`mkdir build`
`cd build`
`cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SYSTEM_EIGEN3=ON -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DCMAKE_INSTALL_PREFIX=${HOME}/Programs/open3d_install ..`
`make -j8`
`make install`

Then build the open3d. This could take a while (30 min or so).
Recommended is to do a local install.
DO NOT BUILD THE PYTHON PACKAGES. Refer to installation instructions for conda.

 
To build the catkin package, you need to add this to your bashrc

export Open3D_DIR="${your install prefix}/lib/cmake/Open3D"

E.g. 
export Open3D_DIR="/home/jelavice/Programs/open3d_install/lib/cmake/Open3D"


Clone the ros conversions and build them in your workspace

`git clone git@github.com:ros-perception/perception_open3d.git`

`catkin build m545_volumetric_mapping`
