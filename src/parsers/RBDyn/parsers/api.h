#pragma once

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
#if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define RBDYN_PARSERS_DLLIMPORT __declspec(dllimport)
#  define RBDYN_PARSERS_DLLEXPORT __declspec(dllexport)
#  define RBDYN_PARSERS_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RBDYN_PARSERS_DLLIMPORT __attribute__((visibility("default")))
#    define RBDYN_PARSERS_DLLEXPORT __attribute__((visibility("default")))
#    define RBDYN_PARSERS_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RBDYN_PARSERS_DLLIMPORT
#    define RBDYN_PARSERS_DLLEXPORT
#    define RBDYN_PARSERS_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RBDYN_PARSERS_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RBDYN_PARSERS_DLLAPI
#  define RBDYN_PARSERS_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RBDYN_PARSERS_EXPORTS
#    define RBDYN_PARSERS_DLLAPI RBDYN_PARSERS_DLLEXPORT
#  else
#    define RBDYN_PARSERS_DLLAPI RBDYN_PARSERS_DLLIMPORT
#  endif // RBDYN_PARSERS_EXPORTS
#  define RBDYN_PARSERS_LOCAL RBDYN_PARSERS_DLLLOCAL
#endif // RBDYN_PARSERS_STATIC
