
## Open3D Installation instructions


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

If the instructions below fail, refer to t he installation manual:
http://www.open3d.org/docs/release/compilation.html

Clone the following repository:  

`git clone --recursive git@bitbucket.org:leggedrobotics/open3d.git`  
`git checkout m545_stable`

Update the submodules:  
`git submodule update --init --recursive`

Create a build directory and build from source:   
`mkdir build`  
`cd build`   
`cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SYSTEM_EIGEN3=ON -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DCMAKE_INSTALL_PREFIX=${HOME}/Programs/open3d_install ..`   
`make -j8`   
`make install`   

If any of the steps fail and you are missing the dependencies, you can try to resolve them with the following instructions:  

http://www.open3d.org/docs/release/compilation.html#install-dependencies  

The script might not install the following deps which then you will have to install manually:  
`sudo apt install libfmt-dev`  
`sudo apt install libglfw3-dev`  

You can also change the install prefix to point to the desired location.
Then build the open3d. This could take a while (30 min or so).
Recommended is to do a local install.
DO NOT BUILD THE PYTHON PACKAGES. Refer to installation instructions for conda.

## Bulding the Package

### Paths
To build the catkin package, you need to add this to your bashrc

export Open3D_DIR="${your install prefix}/lib/cmake/Open3D"

E.g. 
export Open3D_DIR="$HOME/Programs/open3d_install/lib/cmake/Open3D"

### Dependencies
catkin_simple is required in the workspace.
Clone it with:

`git clone git@github.com:catkin/catkin_simple.git`

Build with:
`catkin build m545_volumetric_mapping`
