// Copyright (C) 2015 by Benjamin Chrétien, CNRS-LIRMM.
//
// This file is part of the rbd library.
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

template <typename T>
std::ostream& operator<< (std::ostream& o, const std::vector<T>& vect)
{
  typedef typename std::vector<T>::const_iterator citer_t;

  if (vect.empty ())
    return o << "Empty vector";

  citer_t it = vect.begin ();
  o << "[ " << *it;
  ++it;

  for (; it != vect.end (); ++it)
    o << ", " << *it;
  o << "]";
  return o;
}

template <typename T1, typename T2>
std::ostream& operator<< (std::ostream& o, const std::pair<T1, T2>& p)
{
  return o << "(" << p.first << ", " << p.second << ")";
}

template <typename T1, typename T2>
std::ostream& operator<< (std::ostream& o, const std::map<T1,T2>& m)
{
  typedef typename std::map<T1,T2>::const_iterator citer_t;

  if (m.empty ())
    return o << "{}";

  citer_t it = m.begin ();
  o << "{" << it->first << ": " << it->second;
  ++it;

  for (; it != m.end (); ++it)
    o << ", " << it->first << ": " << it->second;
  return o << "}";
}

template <typename T>
std::ostream& operator<< (std::ostream& o, const Eigen::MatrixBase<T>& matrix)
{
  Eigen::IOFormat ioformat (Eigen::StreamPrecision,
                            Eigen::DontAlignCols,
                            ",", ", ", "(", ")", "(", ")");
  ioformat.rowSpacer = "";
  o << "[";

  // Matrix
  if (matrix.cols () == 1 || matrix.rows () == 1)
  {
    // Vector
    ioformat = Eigen::IOFormat (Eigen::StreamPrecision,
                                Eigen::DontAlignCols,
                                ",", ",", "", "", "(", ")");
    ioformat.rowSpacer = "";
    o << matrix.size ();
  }
  else
    o << matrix.rows () << "," << matrix.cols ();

  o << "]" << matrix.format (ioformat);
  return o;
}
