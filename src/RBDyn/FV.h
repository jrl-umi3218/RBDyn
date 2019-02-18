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
	* Compute the forward velocity of a MultiBody.
	* @param mb MultiBody used has model.
	* @param mbc Use alpha generalized velocity vector, bodyPosW, jointConfig
	* and parentToSon.
	* Fill jointVelocity, bodyVelW and bodyVelB.
	*/
RBDYN_DLLAPI void forwardVelocity(const MultiBody& mb, MultiBodyConfig& mbc);

/**
	* Safe version.
	* @see forwardVelocity.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
RBDYN_DLLAPI void sForwardVelocity(const MultiBody& mb, MultiBodyConfig& mbc);

} // namespace rbd
