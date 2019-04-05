// Copyright (C) 2008-2018 LAAS-CNRS, JRL AIST-CNRS, INRIA.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef @LIBRARY_NAME@_CONFIG_HH
# define @LIBRARY_NAME@_CONFIG_HH

// Package version (header).
# define @LIBRARY_NAME@_VERSION "@PROJECT_VERSION@"
# define @LIBRARY_NAME@_MAJOR_VERSION @PROJECT_VERSION_MAJOR@
# define @LIBRARY_NAME@_MINOR_VERSION @PROJECT_VERSION_MINOR@
# define @LIBRARY_NAME@_PATCH_VERSION @PROJECT_VERSION_PATCH@

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define @LIBRARY_NAME@_DLLIMPORT __declspec(dllimport)
#  define @LIBRARY_NAME@_DLLEXPORT __declspec(dllexport)
#  define @LIBRARY_NAME@_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define @LIBRARY_NAME@_DLLIMPORT __attribute__ ((visibility("default")))
#   define @LIBRARY_NAME@_DLLEXPORT __attribute__ ((visibility("default")))
#   define @LIBRARY_NAME@_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define @LIBRARY_NAME@_DLLIMPORT
#   define @LIBRARY_NAME@_DLLEXPORT
#   define @LIBRARY_NAME@_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef @LIBRARY_NAME@_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define @LIBRARY_NAME@_DLLAPI
#  define @LIBRARY_NAME@_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef @EXPORT_SYMBOL@
#   define @LIBRARY_NAME@_DLLAPI @LIBRARY_NAME@_DLLEXPORT
#  else
#   define @LIBRARY_NAME@_DLLAPI @LIBRARY_NAME@_DLLIMPORT
#  endif // @LIBRARY_NAME@_EXPORTS
#  define @LIBRARY_NAME@_LOCAL @LIBRARY_NAME@_DLLLOCAL
# endif // @LIBRARY_NAME@_STATIC
#endif //! @LIBRARY_NAME@_CONFIG_HH
