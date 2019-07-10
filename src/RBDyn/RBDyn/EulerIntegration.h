/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#ifdef __GNUC__
#  define DO_PRAGMA(x) _Pragma(#x)
#  define PRAGMA_WARNING(x) DO_PRAGMA(GCC warning #x)
#endif //__GNUC__
#ifdef _MSC_VER
#  define __STRINGIFY__(x) #x
#  define __TOSTRING__(x) __STRINGIFY__(x)
#  define PRAGMA_WARNING(x) __pragma(message(__FILE__ "(" __TOSTRING__(__LINE__) ") : warning: " #x))
#endif

PRAGMA_WARNING(EulerIntegration.h is a deprecated header. Please consider using NumericalIntegration.h instead.)

#include <rbdyn/deprecated.hh>

#include "NumericalIntegration.h"

namespace rbd
{

/// Old name for @see jointIntegration
RBDYN_DEPRECATED void eulerJointIntegration(Joint::Type type,
                                            const std::vector<double> & alpha,
                                            const std::vector<double> & alphaD,
                                            double step,
                                            std::vector<double> & q,
                                            double prec = 1e-10)
{
  jointIntegration(type, alpha, alphaD, step, q, prec);
}

/// Old name for @see integration
RBDYN_DEPRECATED void eulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step, double prec = 1e-10)
{
  integration(mb, mbc, step, prec);
}

/// Old name for @see sIntegration
RBDYN_DEPRECATED void sEulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step, double prec = 1e-10)
{
  sIntegration(mb, mbc, step, prec);
}

} // namespace rbd
