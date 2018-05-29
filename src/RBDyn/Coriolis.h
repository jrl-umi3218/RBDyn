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

#include <array>

#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/CoM.h>

namespace rbd
{

struct Block
{
  Block() = default;
  Block(Eigen::DenseIndex startDof, Eigen::DenseIndex startJac, Eigen::DenseIndex length)
    : startDof(startDof), startJac(startJac), length(length)
  {}
  /** Represents a contiguous block of DoFs in a Jacobian */
  Eigen::DenseIndex startDof; /* Start of the block in the full DoF vector */
  Eigen::DenseIndex startJac; /* Start of the block in the jacobian's reduced DoF*/
  Eigen::DenseIndex length; /* Length of the block */
};

using Blocks = std::vector<Block>;

/** Expand a symmetric product of a jacobian by its transpose onto every DoF.
 * @param jac Jacobian whose product will be expanded
 * @param mb Multibody system
 * @param jacMat The product of J^T*J that will be expanded
 */
RBDYN_DLLAPI Eigen::MatrixXd expand(const rbd::Jacobian& jac,
			const rbd::MultiBody& mb,
			const Eigen::MatrixXd& jacMat);

/** Expand a symmetric product of a jacobian by its transpose onto every DoF
 * and accumulate the result
 * @param res Accumulator matrix
 * @see expand
 */
RBDYN_DLLAPI void expandAdd(const rbd::Jacobian& jac,
		const rbd::MultiBody& mb,
		const Eigen::MatrixXd& jacMat,
		Eigen::MatrixXd& res);

/** Compute a compact kinematic path, i.e. the sequence of consecutive blocks
 * of DoF contained in the original path.
 * @param jac Jacobian to be compacted
 * @param mb Multibody system
 */
RBDYN_DLLAPI Blocks compactPath(const rbd::Jacobian& jac,
		const rbd::MultiBody& mb);

/** Expand a symmetric product of a jacobian by its transpose onto every DoF
 * and accumulate the result using a compact representation of the DoF.
 * @param compacPath Blocks representing the compact kinematic path of the Jacobian
 * @param jacMat The matrix containing the product J^T*J to be expanded
 * @param res The accumulator matrix
 **/
RBDYN_DLLAPI void expandAdd(const Blocks& compactPath,
			const Eigen::MatrixXd& jacMat,
			Eigen::MatrixXd& res);

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

};

} // ns rbd
