RBDyn
=====

[![Build Status](https://travis-ci.org/jorisv/RBDyn.svg?branch=master)](https://travis-ci.org/jorisv/RBDyn)

RBDyn provide a set of class and function to model the dynamics of rigid body systems.

This implementation is based on Roy Featherstone's "Rigid Body Dynamics Algorithms" book.

## Dependencies

* C++ compiler with C++11 support
* Eigen 3
* CMake
* Boost
* SpaceVecAlg

For Python bindings:

* Python 2 or 3
* PyBindGen

## Install

```sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=[your install prefix]
make
make install
```

Note: the usual install prefix is `/usr`, but it defaults to `/usr/local`.

If you generate Python bindings on Debian-based distributions (e.g. Ubuntu), you may want to choose the Debian layout for Python packages (e.g. `/usr/lib/python2.7/dist-packages/`) instead of the default one (e.g. `/usr/lib/python2.7/site-packages/`). If this is the case, add the `-DPYTHON_DEB_LAYOUT=ON` flag to the CMake command.

## Pulling git subtree

To update sync cmake or .travis directory with their upstream git repository:

	git fetch git://github.com/jrl-umi3218/jrl-cmakemodules.git master
	git subtree pull --prefix cmake git://github.com/jrl-umi3218/jrl-cmakemodules.git master --squash

	git fetch git://github.com/jrl-umi3218/jrl-travis.git master
	git subtree pull --prefix .travis git://github.com/jrl-umi3218/jrl-travis.git master --squash
