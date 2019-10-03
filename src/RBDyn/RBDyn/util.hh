/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <Eigen/Core>
#include <map>
#include <utility>
#include <vector>

namespace rbd
{
/// \brief Display a vector.
template<typename T>
std::ostream & operator<<(std::ostream &, const std::vector<T> &);

/// \brief Display a pair.
template<typename T1, typename T2>
std::ostream & operator<<(std::ostream &, const std::pair<T1, T2> &);

/// \brief Display a map.
template<typename T1, typename T2>
std::ostream & operator<<(std::ostream &, const std::map<T1, T2> &);

/// \brief Display an Eigen object with the appropriate IOFormat.
template<typename T>
std::ostream & operator<<(std::ostream &, const Eigen::MatrixBase<T> &);

#include "util.hxx"
} // end of namespace rbd
