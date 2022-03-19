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

set -ev

SUDO=${SUDO:=sudo} # SUDO=command in docker (running as root, sudo not available)
if [ "$1" == "assume-yes" ]; then
    APT_CONFIRM="--assume-yes"
else
    APT_CONFIRM=""
fi

dependencies=(
    # Open3D deps
    xorg-dev
    libglu1-mesa-dev
    python3-dev
    # Filament build-from-source deps
    libsdl2-dev
    libc++-7-dev
    libc++abi-7-dev
    ninja-build
    libxi-dev
    # ML deps
    libtbb-dev
    # Headless rendering deps
    libosmesa6-dev
    # RealSense deps
    libudev-dev
    autoconf
    libtool
    #
    libturbojpeg-dev
)

$SUDO apt-get update
for package in "${dependencies[@]}"; do
    $SUDO apt-get install "$APT_CONFIRM" "$package"
done