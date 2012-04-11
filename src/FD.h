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

// includes
// std
#include <vector>

// Eigen
#include <Eigen/Core>

// SpaceVecAlg
#include <SpaceVecAlg>

namespace rbd
{
class MultiBody;
class MultiBodyConfig;

class ForwardDynamics
{
public:
	ForwardDynamics(const MultiBody& mb);

	void forwardDynamics(const MultiBody& mb, MultiBodyConfig& mbc);

	void computeH(const MultiBody& mb, MultiBodyConfig& mbc);
	void computeC(const MultiBody& mb, MultiBodyConfig& mbc);

	const Eigen::MatrixXd& H() const
	{
		return H_;
	}

	const Eigen::VectorXd& C() const
	{
		return C_;
	}

private:
	Eigen::MatrixXd H_;
	Eigen::VectorXd C_;

	std::vector<sva::RBInertia> I_st_;
	std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> F_;
	std::vector<sva::ForceVec> f_;

	std::vector<int> dofPos_;
};

} // namespace rbd

