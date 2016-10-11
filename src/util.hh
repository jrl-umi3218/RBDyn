// Copyright (C) 2015 by Benjamin Chrétien, CNRS-LIRMM.
//
// This file is part of the rsdf library.
//
// rsdf is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// rsdf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with rsdf.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <vector>
#include <utility>
#include <map>

#include <Eigen/Core>

namespace rbd
{
  /// \brief Display a vector.
  template <typename T>
  std::ostream& operator<< (std::ostream&, const std::vector<T>&);

  /// \brief Display a pair.
  template <typename T1, typename T2>
  std::ostream& operator<< (std::ostream&, const std::pair<T1, T2>&);

  /// \brief Display a map.
  template <typename T1, typename T2>
  std::ostream& operator<< (std::ostream&, const std::map<T1, T2>&);

  /// \brief Display an Eigen object with the appropriate IOFormat.
  template <typename T>
  std::ostream& operator<< (std::ostream&, const Eigen::MatrixBase<T>&);

  #include "util.hxx"
} // end of namespace rbd
