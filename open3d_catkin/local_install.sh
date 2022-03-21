#!/bin/bash

#=============================================================================
# Copyright (C) 2022, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# Authors: Julian Nubert, nubertj@ethz.ch
#          Edo Jelavic, jelavice@ethz.ch
#          Marco Tranzatto, marcot@ethz.ch
#=============================================================================

CMAKE_SOURCE_DIR=$1

if [ ! -d "$HOME/Programs" ]; then
  mkdir -p "$HOME/Programs"
fi

if [ ! -d "$HOME/Programs/open3d_install" ]; then
  mkdir -p "$HOME/Programs/open3d_install"
fi

cd $CMAKE_SOURCE_DIR/tmp
make install
#echo "export Open3D_DIR='$HOME/Programs/open3d_install/lib/cmake/Open3D'" >> ~/.bashrc
#source ~/.bashrc

