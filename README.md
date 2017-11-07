RBDyn
=====

[![License LGPL 3](https://img.shields.io/badge/license-LGPLv3-green.svg)](http://www.gnu.org/licenses/lgpl-3.0.txt)
[![Build Status](https://travis-ci.org/jrl-umi3218/RBDyn.svg?branch=master)](https://travis-ci.org/jrl-umi3218/RBDyn)
[![AppVeyor status](https://ci.appveyor.com/api/projects/status/k2l715sc47t9u411/branch/master?svg=true)](https://ci.appveyor.com/project/gergondet/rbdyn/branch/master)

RBDyn provide a set of class and function to model the dynamics of rigid body systems.

This implementation is based on [Roy Featherstone Rigid Body Dynamics Algorithms book](http://www.springer.com/fr/book/9780387743141) and other state of the art publications.

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

Installing
------


### Ubuntu 14.04 and 16.04 binary ppa install

Use the [multi-contact-unstable](https://launchpad.net/~pierre-gergondet+ppa/+archive/ubuntu/multi-contact-unstable) ppa:
```bash
sudo add-apt-repository ppa:pierre-gergondet+ppa/multi-contact-unstable
sudo apt-get update
sudo apt-get install librbdyn-dev librbdyn-doc
```

### Homebrew OS X install

Install from the command line using [Homebrew](brew.sh):

```bash
# install homebrew package manager
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
# install caskroom application manager
brew install caskroom/cask/brew-cask
# tap homebrew-science package repository
brew tap homebrew/science
# tap ahundt-robotics repository
brew tap ahundt/robotics
# install tasks and all its dependencies
brew install rbdyn
```

## Manually build from source

#### Dependencies

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

 * [Cython](cython.readthedocs.io) = 0.20
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython) (to use the python binding)

#### Building

```sh
git clone --recursive https://github.com/jrl-umi3218/RBDyn
cd RBDyn
mkdir _build
cd _build
cmake [options] ..
make && make intall
```

Where the main options are:

 * `-DCMAKE_BUIlD_TYPE=Release` Build in Release mode
 * `-DCMAKE_INSTALL_PREFIX=some/path/to/install` default is `/usr/local`
 * `-DPYTHON_BINDING=ON` Build the python binding
 * `-DUNIT_TESTS=ON` Build unit tests.
 * `-DPYTHON_DEB_LAYOUT=OFF` install python library in `site-packages` (ON will install in `dist-packages`)

### Arch Linux

You can use the following [AUR package](https://aur.archlinux.org/packages/rbdyn-git).


Pulling git subtree
-------

To update sync cmake or .travis directory with their upstream git repository:

	git fetch git://github.com/jrl-umi3218/jrl-cmakemodules.git master
	git subtree pull --prefix cmake git://github.com/jrl-umi3218/jrl-cmakemodules.git master --squash

	git fetch git://github.com/jrl-umi3218/jrl-travis.git master
	git subtree pull --prefix .travis git://github.com/jrl-umi3218/jrl-travis.git master --squash
