RBDyn
=====

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Hosted By: Cloudsmith](https://img.shields.io/badge/OSS%20hosting%20by-cloudsmith-blue?logo=cloudsmith)](https://cloudsmith.com)
[![CI](https://github.com/jrl-umi3218/RBDyn/workflows/CI%20of%20RBDyn/badge.svg?branch=master)](https://github.com/jrl-umi3218/RBDyn/actions?query=workflow%3A%22CI+of+RBDyn%22)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/RBDyn/doxygen/HEAD/index.html)

RBDyn provides a set of classes and functions to model the dynamics of rigid body systems.

This implementation is based on [Roy Featherstone Rigid Body Dynamics Algorithms book](http://www.springer.com/fr/book/9780387743141) and other state of the art publications.

Installing
------

## Ubuntu LTS (16.04, 18.04, 20.04)

You must first setup our package mirror:

```
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash
```

You can also choose the head mirror which will have the latest version of this package:

```
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/head/setup.deb.sh' \
  | sudo -E bash
```

You can then install the package:

```bash
sudo apt install librbdyn-dev python-rbdyn python3-rbdyn
```

## vcpkg

Use the registry available [here](https://github.com/mc-rtc/vcpkg-registry/)


## Homebrew OS X install

Install from the command line using [Homebrew](brew.sh):

```bash
# Use mc-rtc tap
brew tap mc-rtc/mc-rtc
# install RBDyn and its Python bindings
brew install rbdyn
```

## Manually build from source

### Dependencies

To compile you need the following tools:

 * [Git]()
 * [CMake]() >= 2.8
 * [pkg-config]()
 * [doxygen]()
 * [g++]() >= 4.7 (for C++11 support)
 * [Boost](http://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html) >= 1.49
 * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2
 * [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)

For Python bindings:

 * [Cython](cython.readthedocs.io) >= 0.20
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython) (to use the python binding)

### Building

```sh
git clone --recursive https://github.com/jrl-umi3218/RBDyn
cd RBDyn
mkdir _build
cd _build
cmake [options] ..
make && make intall
```

#### CMake options

By default, the build will use the `python` and `pip` command to install the bindings for the default system version (this behaviour can be used to build the bindings in a given virtualenv). The following options allow to control this behaviour:

 * `PYTHON_BINDING` Build the python binding (ON/OFF, default: ON)
 * `PYTHON_BINDING_FORCE_PYTHON2`: use `python2` and `pip2` instead of `python` and `pip`
 * `PYTHON_BINDING_FORCE_PYTHON3`: use `python3` and `pip3` instead of `python` and `pip`
 * `PYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3`: builds two sets of bindings one with `python2` and `pip2`, the other with `python3` and `pip3`
 * `BUILD_TESTING` Enable unit tests building (ON/OFF, default: ON)

### Arch Linux

You can use the following [AUR package](https://aur.archlinux.org/packages/rbdyn-git).

Documentation
-----

Features:
 * Kinematics tree Kinematics and Dynamics algorithm C++11 implementation
 * Use Eigen3 and [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg) library
 * Free, Spherical, Planar, Cylindrical, Revolute, Prismatic joint support
 * Translation, Rotation, Vector, CoM, Momentum Jacobian computation
 * Inverse Dynamics, Forward Dynamics
 * Inverse Dynamic Identification Model (IDIM)
 * Kinematics tree body merging/filtering
 * Kinematics tree base selection
 * Python binding

To make sure that RBDyn works as intended, unit tests are available for each algorithm.
Besides, the library has been used extensively to control humanoid robots such as HOAP-3, HRP-2, HRP-4 and Atlas.

A short tutorial is available [here](https://github.com/jorisv/sva_rbdyn_presentation/blob/master/presentation_release.pdf).

The [SpaceVecAlg and RBDyn tutorial](https://github.com/jorisv/sva_rbdyn_tutorials) is also a big ressource to understand how to use RBDyn by providing a lot of IPython Notebook that will present real use case.

A doxygen documentation is available [online](https://jrl-umi3218.github.io/RBDyn).
