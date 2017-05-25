# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of RBDyn.
#
# RBDyn is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RBDyn is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

from c_rbdyn cimport *
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool

cdef extern from "rbdyn_wrapper.hpp" namespace "rbd":
  string BodyToString(const Body &)
  string JointToString(const Joint &)
  vector[double] ZeroParam(JointType)
  vector[double] ZeroDof(JointType)
  ConfigConverter* ConfigConverterConstructor(const MultiBody&, const MultiBody&) except +
  MultiBody& const_cast_mb(const MultiBody&)
  MultiBodyConfig& const_cast_mbc(const MultiBodyConfig&)
  MultiBodyGraph& const_cast_mbg(const MultiBodyGraph&)
  void dv_set_item(vector[double] & v, unsigned int idx, double value)
  void dvv_set_item(vector[vector[double]] & v, unsigned int idx, const
      vector[double] & value)
  void mbcv_set_item(vector[MultiBodyConfig]&, unsigned int, const MultiBodyConfig&)
