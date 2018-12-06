// Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
//
// This file is part of RBDyn.
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
	* Compute the forward kinematic of a MultiBody.
	* @param mb MultiBody used has model.
	* @param mbc Use q generalized position vector. Fill bodyPosW, jointConfig, motionSubspace
	* and parentToSon.
	*/
RBDYN_DLLAPI void forwardKinematics(const MultiBody& mb, MultiBodyConfig& mbc);

/**
	* Safe version.
	* @see forwardKinematics.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
RBDYN_DLLAPI void sForwardKinematics(const MultiBody& mb, MultiBodyConfig& mbc);

} // namespace rbd
