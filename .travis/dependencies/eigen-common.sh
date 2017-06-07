#!/bin/bash
#
# Setup Eigen
#
. `dirname $0`/../common.sh

EIGEN_VERSION=$1
EIGEN_HASH=$2

# Checkout Eigen
cd "$build_dir"
wget --quiet "http://bitbucket.org/eigen/eigen/get/${EIGEN_VERSION}.tar.gz"
tar xzf ${EIGEN_VERSION}.tar.gz
cd "$build_dir/eigen-eigen-${EIGEN_HASH}/"
mkdir -p "$build_dir/eigen-eigen-${EIGEN_HASH}/_build"
cd "$build_dir/eigen-eigen-${EIGEN_HASH}/_build"

# Build, make and install Eigen
cmake .. -DCMAKE_INSTALL_PREFIX:STRING="$install_dir"
make
make install

# Check install
pkg-config --modversion "eigen3 >= ${EIGEN_VERSION}"
pkg-config --cflags "eigen3 >= ${EIGEN_VERSION}"
