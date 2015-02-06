jrl-travis
==========

This repository gathers build scripts to be used by Travis during
continuous integration.

Currently, two ways of building the software are supported:

 1. head of the development branch (`master`) is built against the
    development versions of its dependencies that are developed by us.
    If the compilation and the tests succeed, the online documentation
    is automatically uploaded so that it stays up-to-date. The code
    coverage statistics are also uploaded to the [coveralls.io][] website
    where it is displayed.

 1. the Debian branch is using git-buildpackage and git-pbuilder to
    build Debian packages for both Debian unstable (Sid) and the
    following Ubuntu releases:
     * Ubuntu 13.10 (Saucy)
     * Ubuntu 13.04 (Raring)
     * Ubuntu 12.10 (Quantal)
     * Ubuntu 12.04 LTS (Precise)
    If the Ubuntu builds succeed, the Debian source package (`*.dsc`)
    is uploaded to the PPA associated with the project to update the
    snapshot PPA. Launchpad will build once more the project and host
    the generated Debian package files. The two supported architecture
    are `amd64` and `i386`.

The dispatch between these two modes is realized by testing whether or
not a `debian/` directory exists in the repository root-level
directory.

Features
--------

Unless Python or Ruby packages, streamlined C++ application deployment
is extremely challenging. Portability issues, compilation flags,
interoperability between packages is notable difficult to ensure.

This submodule contains a set of shell scripts which can be used to
help Travis integrate with various services and tools useful for any
C++ project.

In particular:

 * Setup dependencies both from apt and from source. In the source
   case, dependencies are built and installed in a local prefix.
 * Compile and test the software
 * Using the [jrl-cmakemodules][]:
   * Run a CMake compatible `make distcheck` (build a release tarball
     and the rebuild the whole project from it.
   * Upload the latest Doxygen documentation to GitHub pages.
 * Integrate with [cppcheck][], [coverity][] to run
   static analysis on the code and detect issues.
 * Use [coveralls.io][] to track test coverage.
 * Annotate (see [git-notes][]) successful builds with dependencies
   commit id to be able to reproduce again the exact same successful
   build.

Additionally, Debian builds can be realized in a pbuilder sandbox to
produce Debian packages for various distributions.

Both [`linux`](http://docs.travis-ci.com/user/ci-environment/) and
[`osx`](http://docs.travis-ci.com/user/osx-ci-environment/) environments
are supported. In your `.travis.yml` file, you can set:

```
os:
  - linux
  - osx
```

[jrl-cmakemodules]: https://github.com/jrl-umi3218/jrl-cmakemodules
[cppcheck]: http://cppcheck.sourceforge.net/
[coveralls.io]: https://coveralls.io/

Development branch
------------------

### Before Install

In this case, the complex part is that we have to obtain the project
current dependencies both from APT and those which must be compiled
from source.

The following environment variables are defining the project
dependencies or parameters:

 * `APT_DEPENDENCIES` is passed directly to `apt-get install` (`linux` only).
 * `HOMEBREW_DEPENDENCIES` is passed directly to `brew install` (`osx` only).
 * `GIT_DEPENDENCIES` contains the name of the GitHub repositories that
   will be built from source. For instance: `jrl-umi3218/jrl-mathtools
   stack-of-tasks/dynamic-graph` is a valid chain. Please note that
   you have to list your dependencies in the correct order if some of the
   dependencies depend on other packages compiled from source which must
   be installed first.
 * `MASTER_PPA` can contain a list of PPA which are needed for this project
   to compile (`linux` only). For example:

   ```sh
   # Use the latest Boost release
   MASTER_PPA="boost-latest/ppa"
   ```
   
 * If `COVERITY_TOKEN` is set, [coverity][] integration will be setup
   (`linux` only).
 * `LCOV_IGNORE_RULES` contains ignore rules for the [coveralls.io][]
   report (`linux` only). It should be provided in the form:

   ```sh
   # Ignore all paths containing "foo" or "bar"
   LCOV_IGNORE_RULES="*foo* *bar*"
   # This will lead to:
   # lcov --remove coverage.info '*foo*' '*bar*' -o coverage.info
   ```


### Build

The build step in this case is just configuring the package, building
the package, installing it and running the test suite.


Again, if `COVERITY_TOKEN` is set on `linux`, the `cov-int` tool will
be used to generate a report which will be uploaded to the Coverity
website if the build is successful.


### After success

The last step is uploading the documentation in case of success. To
achieve this, the Travis build machine must obtain write access to the
project repository. To do so, you must provide a push-capable URI in
the `GIT_PUSH_URI`. One way is to use a OAuth token. Go to your
account settings, Applications, Personal Access Token and click on
`Create New Token`. You can then use the `travis` command-line client
to encrypt the environment variable:

```sh
$ travis encrypt GH_PUSH_URI=https://<USERNAME>@<YOUR OAUTH TOKEN>:<ORGANIZATION>/<REPO> --add
```

This has to be run in the project root-level directory. Do _not_ copy
encrypted strings from one project to another. Each repository has its
own key a repository B cannot unencrypt a secure variable encrypted in
repository A. Redo the operation for each repository.

Any other push-capable URI can work but it must not trigger an
interactive behavior (i.e. waiting for the user to input its password
for instance). _Never_ use directly your account password as this
password may be leaked in the Travis log if a bug in these scripts
appear. It would compromise your entire account. On the opposite, it
is easy to revoke an OAuth token.

_Be careful:_ the `gh-pages` must never run the build in this
repository. As we are committing to this branch, using these scripts
on this branch may result in an infinite number of successive build
triggered by each documentation update.


If `COVERITY_TOKEN` is set, a tarball containing the results of the
build is created and uploaded to the website.


To finish, a `git notes` is used to annotate the commit with the build
result. A string containing the commit id of all the Git dependencies
is computed and inserted into the note so that one can exactly
retrieve the combinaison of software which allowed a successful build.


Debian branch
-------------

### Before Install

The `before_install` script will first create a pbuilder sandbox
matching the current target distribution. The target distribution is
controlled by the `DIST` environment variable. This environment
variable is put into the build matrix so that we can build the
software for each version of Debian and Ubuntu separately.

`DEBIAN_PPA` can contain a list of PPA which are needed for this
project to compile. This will setup the PPA _inside_ the pbuilder
sandbox.

All dependencies are installed using `apt-get install`. This is done
automatically by using the `Build-Depends` field of the
`debian/control` file.


### Build

The `build` step first generate a fake entry into the
`debian/changelog` file indicating that this build is a snapshot.

`git-buildpackage` is then called to try building the package into the
pbuilder sandbox. If it success, `git-buidpackage` is called once more
to generate a source package.

To sign the package so that it can be uploaded to a remote location
such as Launchpad, a dedicated key is used. The key public and private
data are provided by this repository. This key (5AE5CD75) is protected
by a passphrase. You have to set the key id using the `DEBSIGN_KEYID`
environment variable while the passphrase is stored in the
`GNUPG_PASSPHRASE` secured environment variable. To generate the
entry, run the following command:

```sh
$ travis encrypt GNUPG_PASSPHRASE=<YOUR KEY PASSPHRASE> --add
```


### After Success

If the two previous step were successful, the Debian source package is
uploaded to launchpad. The `PPA_URI` allow to control in which PPA the
package will be uploaded. In the case of Debian unstable (Sid), the
package is not uploaded as the snapshot cannot enter the official
repository. Of course, the key used to sign the package must be
allowed to upload packages.


### Debian packaging notes

It is plausible that the content of this repository will not match the
one in the release. Add the following `debian/source/options` file if
necessary:

```sh
extend-diff-ignore = '^\.travis'
```

It will tell `dpkg-source` to ignore all the modifications in this
directory. This is safe because the content of this build is for
Travis only and should _never_ end up changing the final Debian
package in any way.


Tips and Tricks
---------------

### Using this repository in your project

This repository is being meant to be used as submodule. In your
project root directory, please run:

```sh
$ git submodule add git://github.com/jrl-umi3218/jrl-travis.git .travis
```

You may want to fork the repository first if your project need to be
compiled in a particular way.

You can use the `travis.yml.in` file as a template for your project:

```sh
$ cp .travis/travis.yml.in .travis.yml
```

All the fields `@FOO@` must be replaced by their real value.


### Looking for successful builds

The `after_success` script is automatically adding a [git-notes][] to
all the successful build. These notes will not be displayed by default
though. To retrieve the notes and display them:

```sh
$ git fetch origin refs/notes/travis:refs/notes/travis
$ git log --notes=travis --show-notes
```

You should see an output like this one:

```
commit 8e78835cdbbf89f099f394dce9f6083dc802c994
Author: Thomas Moulard <thomas.moulard@gmail.com>
Date:   Mon Sep 16 13:37:19 2013 +0900

    Synchronize

    Notes (travis):
        Successful build.
	----

	Dependencies commit id:
```

The note also contain the commit id of all the dependencies installed
from source to allow you to go back to the exact setup used by Travis
in this build.

[git-notes]: https://www.kernel.org/pub/software/scm/git/docs/git-notes.html
[coverity]: https://scan.coverity.com/

License
-------

The whole content of this repository is licensed under BSD. See
[COPYING](COPYING) for more information.


Authors and Credits
-------------------

 * Thomas Moulard <thomas.moulard@gmail.com> (maintainer)
 * Benjamin Chrétien <chretien@lirmm.fr> (developer)


We would like to express our gratitude to [Travis
CI](http://www.travis-ci.org) for building this incredible continuous
integration tool.
