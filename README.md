RBDyn
=====

[![Build Status](https://travis-ci.org/jorisv/RBDyn.svg?branch=master)](https://travis-ci.org/jorisv/RBDyn)

RBDyn provide a set of class and function to model the dynamics of rigid body systems.

This implementation is based on [Roy Featherstone Rigid Body Dynamics Algorithms book](http://www.springer.com/fr/book/9780387743141) and other state of the art publications.

Documentation
-----

Features:
 * Kinematics tree Kinematics and Dynamics algorithm C++11 implementation
 * Use Eigen3 and [SpaceVecAlg](https://github.com/jorisv/SpaceVecAlg) library
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

Installing
------

### Manual

#### Dependencies

To compile you need the following tools:

 * [Git]()
 * [CMake]() >= 2.8
 * [pkg-config]()
 * [doxygen]()
 * [g++]() >= 4.7 (for C++11 support)
 * [Boost](http://www.boost.org/doc/libs/1_58_0/more/getting_started/unix-variants.html) >= 1.49
 * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2
 * [PyBindGen](https://launchpad.net/pybindgen) = 0.16
 * [Eigen3ToPython](https://github.com/jorisv/Eigen3ToPython) (to use the python binding)
 * [SpaceVecAlg](https://github.com/jorisv/SpaceVecAlg)

#### Building

```sh
git clone --recursive https://github.com/jorisv/RBDyn
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
 * `-DPYTHON_DEB_LAYOUT=OFF` install python library in `site-packages` (ON will install in `dist-packages`

### Arch Linux

You can use the following [AUR package](https://aur.archlinux.org/packages/rbdyn-git).


Pulling git subtree
-------

To update sync cmake or .travis directory with their upstream git repository:

	git fetch git://github.com/jrl-umi3218/jrl-cmakemodules.git master
	git subtree pull --prefix cmake git://github.com/jrl-umi3218/jrl-cmakemodules.git master --squash

	git fetch git://github.com/jrl-umi3218/jrl-travis.git master
	git subtree pull --prefix .travis git://github.com/jrl-umi3218/jrl-travis.git master --squash
