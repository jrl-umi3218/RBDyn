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

namespace rbd
{
class MultiBody;
class MultiBodyConfig;

/**
	* Compute the forward acceleration of a MultiBody.
	* @param mb MultiBody used has model.
	* @param mbc Use alphaD generalized acceleration vector, jointVelocity,
	* parentToSon and bodyVel.
	* Fill bodyAccB.
	*/
void forwardAcceleration(const MultiBody& mb, MultiBodyConfig& mbc);

/**
	* Safe version.
	* @see forwardAcceleration.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
void sForwardAcceleration(const MultiBody& mb, MultiBodyConfig& mbc);

} // namespace rbd
