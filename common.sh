# -*- sh-mode -*
# This should be sourced, not called.
set -e

# Directories.
root_dir=`pwd`

build_dir="/tmp/_travis/build"
install_dir="/tmp/_travis/install"

echo "root_dir: " $root_dir
echo "build_dir: " $build_dir
echo "install_dir: " $install_dir

# osx support is still in beta
if `test x${TRAVIS_OS_NAME} = x`; then
    export TRAVIS_OS_NAME=linux
fi

# Shortcuts.
git_clone="git clone --quiet --recursive"

# Setup environment variables.
if [ -d /opt/ros ]; then
  . /opt/ros/${ROS_DISTRO}/setup.sh
fi

export LD_LIBRARY_PATH="$install_dir/lib:$LD_LIBRARY_PATH"
export LTDL_LIBRARY_PATH="$install_dir/lib:$LTDL_LIBRARY_PATH"
export PKG_CONFIG_PATH="$install_dir/lib/pkgconfig:$PKG_CONFIG_PATH"

if [[ ${TRAVIS_OS_NAME} = linux ]]; then
    export LD_LIBRARY_PATH="$install_dir/lib/`dpkg-architecture -qDEB_BUILD_MULTIARCH`:$LD_LIBRARY_PATH"
    export LTDL_LIBRARY_PATH="$install_dir/lib/`dpkg-architecture -qDEB_BUILD_MULTIARCH`:$LTDL_LIBRARY_PATH"
    export PKG_CONFIG_PATH="$install_dir/lib/`dpkg-architecture -qDEB_BUILD_MULTIARCH`/pkgconfig:$PKG_CONFIG_PATH"
fi

if type "python" > /dev/null; then
    pythonsite_dir=`python -c "import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], 'site-packages']))"`
    export PYTHONPATH="$install_dir/$pythonsite_dir:$PYTHONPATH"
fi

if [[ ${TRAVIS_OS_NAME} = osx ]]; then
    # Since default gcc on osx is just a front-end for LLVM...
    if [[ ${CC} = gcc ]]; then
	export CXX=g++-4.8
	export CC=gcc-4.8
    fi
fi

# Make cmake verbose.
export CMAKE_VERBOSE_MAKEFILE=1
export CTEST_OUTPUT_ON_FAILURE=1

# Create layout.
mkdir -p "$build_dir"
mkdir -p "$install_dir"

# Add verbose handling
. .travis/verbose_errors.sh
