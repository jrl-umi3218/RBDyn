// Copyright 2012-2018 CNRS-UM LIRMM, CNRS-AIST JRL
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

#include <array>

#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/CoM.h>
#include <RBDyn/Jacobian.h>

namespace rbd
{

/**
 * Computation of the Coriolis effects matrix on a multibody. The Coriolis
 * factorization is not unique, we use the formulation of M. Bjerkend and
 * K. Pettersen in "A new Coriolis matrix factorization", 2012
 * NB: ForwardDynamics::C() directly computes the product of this matrix with qd.
 * This C*qd is unique, but C itself is not.
 */
class RBDYN_DLLAPI Coriolis
{
public:
	/** Initialize the required structures
	 * @param mb Multibody system
	 */
	Coriolis(const rbd::MultiBody& mb);


	/** Compute the matrix C of Coriolis effects.
	 * @param mb Multibody system
	 * @param mbc Multibody configuration associated to mb
	 */
	const Eigen::MatrixXd& coriolis(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

private:
	std::vector<rbd::Jacobian> jacs_;
	std::vector<Blocks> compactPaths_;
	Eigen::MatrixXd coriolis_;
	Eigen::MatrixXd res_;
};

} // ns rbd
