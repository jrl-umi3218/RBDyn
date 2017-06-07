# -*- sh-mode -*
# This should be sourced, not called.
set -e

########################################
#        -- VERBOSE HANDLING --        #
########################################

# More verbose handling for 'set -e'.
#
# Show a traceback if we're using bash, otherwise just a message.
# Downloaded from: https://gist.github.com/kergoth/3885825
on_exit () {
    ret=$?
    case $ret in
        0)
            ;;
        *)
            echo >&2 "Exiting with $ret from a shell command"
            ;;
    esac
}

on_error () {
    local ret=$?
    local FRAMES=${#BASH_SOURCE[@]}

    echo >&2 "Traceback (most recent call last):"
    for ((frame=FRAMES-2; frame >= 0; frame--)); do
        local lineno=${BASH_LINENO[frame]}

        printf >&2 '  File "%s", line %d, in %s\n' "${BASH_SOURCE[frame+1]}" "$lineno" "${FUNCNAME[frame+1]}"
        sed >&2 -n "${lineno}s/^[ 	]*/    /p" "${BASH_SOURCE[frame+1]}" || true
    done
    printf >&2 "Exiting with %d\n" "$ret"
    exit $ret
}

case "$BASH_VERSION" in
    '')
        trap on_exit EXIT
        ;;
    *)
        set -o errtrace
        trap on_error ERR
        ;;
esac

########################################
#        -- GLOBAL UTILITIES --        #
########################################

# git_dependency_parsing
# ----------------------
#
# From an entry in GIT_DEPENDENCIES variable set git_dep, git_dep_uri and
# git_dep_branch in the environment
# For example given the input "jrl-umi3218/jrl-travis" the following variables
# are defined in the environment:
# - git_dep jrl-umi3218/jrl-travis
# - git_dep_uri git://github.com/jrl-umi3218/jrl-traviss
# - git_dep_branch master
# Or, given the input git@github.com:jrl-umi3218/jrl-travis#thebranch
# - git_dep jrl-umi3218/jrl-travis
# - git_dep_uri git@github.com:jrl-umi3218/jrl-travis
# - git_dep_branch thebranch
# The second (optional) argument allows to defined the default branch (defaults
# to master)
git_dependency_parsing()
{
  _input=$1
  export git_dep=${_input%%#*}
  export git_dep_branch=${_input##*#}
  if [ "$git_dep_branch" == "$git_dep" ]; then
    if [ -e "$2" ]; then
      export git_dep_branch=$2
    else
      export git_dep_branch="master"
    fi
  fi
  git_dep_uri_base=${git_dep%%:*}
  if [ "$git_dep_uri_base" == "$git_dep" ]; then
    export git_dep_uri="git://github.com/$git_dep"
  else
    export git_dep_uri=$git_dep
    export git_dep=${git_dep##*:}
  fi
}


########################################
#    -- ENVIRONMENT MANIPULATION --    #
########################################

_gitlab_setup_ci_vars()
{
  export CI_REQUIRE_SUDO=false
  export CI_PULL_REQUEST=false #FIXME Can it be provided by gitlab?
  export CI_REPO_SLUG=`echo ${CI_PROJECT_DIR}|sed -e's@/builds/@@'`
  export CI_BRANCH=${CI_BUILD_REF_NAME}
  export CI_OS_NAME=${CI_OS_NAME:-linux}
}

_travis_setup_ci_vars()
{
  export CI_REQUIRE_SUDO=${CI_REQUIRE_SUDO:-true}
  export CI_PULL_REQUEST=${TRAVIS_PULL_REQUEST}
  export CI_REPO_SLUG=${TRAVIS_REPO_SLUG}
  export CI_BRANCH=${TRAVIS_BRANCH}
  export CI_OS_NAME=${TRAVIS_OS_NAME:-linux}
}

# _setup_ci_vars
# --------------
#
# Setup CI_* variables based on the CI type
_setup_ci_vars()
{
  # Check which CI tool we are using, default to travis
  export CI_TOOL=${CI_TOOL:-travis}

  if [ $CI_TOOL = travis ]; then
    _travis_setup_ci_vars
  else
    _gitlab_setup_ci_vars
  fi
}

# _setup_sudo_cmd
# ---------------
#
# Setup SUDO_CMD based on CI configuration
_setup_sudo_cmd()
{
  if [ ${CI_REQUIRE_SUDO} = false ]; then
    export SUDO_CMD=''
  else
    export SUDO_CMD='sudo'
  fi
}

# _setup_ros
# ----------
#
# Setup ROS environment if present on the system
_setup_ros()
{
  if [ -f /opt/ros/${ROS_DISTRO}/setup.sh ]; then
    . /opt/ros/${ROS_DISTRO}/setup.sh
  fi
}

# _setup_env_vars
# ---------------
#
# Setup environment variables
_setup_env_vars()
{
  export LD_LIBRARY_PATH="$install_dir/lib:$LD_LIBRARY_PATH"
  export LTDL_LIBRARY_PATH="$install_dir/lib:$LTDL_LIBRARY_PATH"
  export PKG_CONFIG_PATH="$install_dir/lib/pkgconfig:$install_dir/share/pkgconfig:$PKG_CONFIG_PATH"
  if type "python" > /dev/null; then
    pythonsite_dir=`python -c "import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], 'site-packages']))"`
    export PYTHONPATH="$install_dir/$pythonsite_dir:$PYTHONPATH"
  fi

}

# _setup_linux_env
# ----------------
#
# Environment setup specific to linux
_setup_linux_env()
{
  export LD_LIBRARY_PATH="$install_dir/lib/`dpkg-architecture -qDEB_BUILD_MULTIARCH`:$LD_LIBRARY_PATH"
  export LTDL_LIBRARY_PATH="$install_dir/lib/`dpkg-architecture -qDEB_BUILD_MULTIARCH`:$LTDL_LIBRARY_PATH"
  export PKG_CONFIG_PATH="$install_dir/lib/`dpkg-architecture -qDEB_BUILD_MULTIARCH`/pkgconfig:$PKG_CONFIG_PATH"
}

# _setup_osx_env
# ----------------
#
# Environment setup specific to OSX
_setup_osx_env()
{
  # Since default gcc on osx is just a front-end for LLVM...
  if [[ ${CC} = gcc ]]; then
    export CXX=g++-4.8
    export CC=gcc-4.8
  fi

  export DYLD_LIBRARY_PATH="$install_dir/lib:$DYLD_LIBRARY_PATH"
  export LTDL_LIBRARY_PATH="$install_dir/lib:$LTDL_LIBRARY_PATH"
  export PKG_CONFIG_PATH="$install_dir/lib/pkgconfig:$PKG_CONFIG_PATH"
}

# setup_ci_env
# ------------
#
# Setup the CI and environment variables
setup_ci_env()
{
  _setup_ci_vars
  _setup_sudo_cmd
  _setup_ros
  _setup_env_vars
  if [[ ${CI_OS_NAME} = linux ]]; then
    _setup_linux_env
  fi
  if [[ ${CI_OS_NAME} = osx ]]; then
    _setup_osx_env
  fi
}

# Directories.
root_dir=`pwd`
build_dir="/tmp/_ci/build"
install_dir="/tmp/_ci/install"

echo "root_dir: " $root_dir
echo "build_dir: " $build_dir
echo "install_dir: " $install_dir

# Shortcuts.
git_clone="git clone --quiet --recursive"

# Setup all variables needed by the CI scripts
setup_ci_env

# Make cmake verbose.
export CMAKE_VERBOSE_MAKEFILE=1
export CTEST_OUTPUT_ON_FAILURE=1

# Add default DO_*_ON_BRANCH if needed
if [ -z ${DO_COVERAGE_ON_BRANCH+x} ]; then
  export DO_COVERAGE_ON_BRANCH=${CI_BRANCH}
fi

if [ -z ${DO_CPPCHECK_ON_BRANCH+x} ]; then
  export DO_CPPCHECK_ON_BRANCH=${CI_BRANCH}
fi

# Create layout.
mkdir -p "$build_dir"
mkdir -p "$install_dir"
