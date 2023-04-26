/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#ifdef __GNUC__
#  define RBDYN_DO_PRAGMA(x) _Pragma(#x)
#  define RBDYN_PRAGMA_WARNING(x) RBDYN_DO_PRAGMA(GCC warning #x)
#endif //__GNUC__
#ifdef _MSC_VER
#  define __RBDYN_STRINGIFY__(x) #x
#  define __RBDYN_TOSTRING__(x) __RBDYN_STRINGIFY__(x)
#  define RBDYN_PRAGMA_WARNING(x) __pragma(message(__FILE__ "(" __RBDYN_TOSTRING__(__LINE__) ") : warning: " #x))
#endif

#ifndef rbdyn_EXPORTS
RBDYN_PRAGMA_WARNING(EulerIntegration.h is a deprecated header.Use NumericalIntegration.h instead.)
#endif

#include <rbdyn/deprecated.hh>

#include "NumericalIntegration.h"

namespace rbd
{

/// Old name for @see jointIntegration
RBDYN_DEPRECATED RBDYN_DLLAPI void eulerJointIntegration(Joint::Type type,
                                                         const std::vector<double> & alpha,
                                                         const std::vector<double> & alphaD,
                                                         double step,
                                                         std::vector<double> & q);

/// Old name for @see integration
RBDYN_DEPRECATED RBDYN_DLLAPI void eulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step);

/// Old name for @see sIntegration
RBDYN_DEPRECATED RBDYN_DLLAPI void sEulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step);

} // namespace rbd
