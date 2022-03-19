# Open3d_catkin
## Dependencies
### Cmake
Version >3.18 is required to build open3d.
For installation, do:
* Download tar archive from https://cmake.org/download/
* ```tar -xf cmake-3.23.0-rc4.tar.gz```
* Then do
```
cd cmake-3.23.0-rc4.tar.gz
./configure
make -j12
sudo make install
```
### Other Open3D Dependencies
Execute either the original instatllation script from open3d: 
[script](https://github.com/isl-org/Open3D/blob/v0.13.0/util/install_deps_ubuntu.sh),
or our fetched version via
```sudo ./install_Deps.sh```