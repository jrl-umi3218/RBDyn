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

// associated header
#include "CoM.h"

// RBDyn
#include "Jacobian.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"

namespace rbd
{

Eigen::Vector3d computeCoM(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	Vector3d com = Vector3d::Zero();

	for(std::size_t i = 0; i < mb.nrBodies(); ++i)
	{
		double mass = bodies[i].inertia().mass();
		Vector3d comT = bodies[i].inertia().momentum()/mass;

		com += (sva::PTransform(comT)*mbc.bodyPosW[i]).translation()*mass;
	}

	return com/mb.nrBodies();
}



CoMJacobianDummy::CoMJacobianDummy()
{}


CoMJacobianDummy::CoMJacobianDummy(const MultiBody& mb):
  jac_(6, mb.nrDof()),
  jacFull_(6, mb.nrDof()),
  jacVec_(mb.nrBodies())
{
  using namespace Eigen;

	for(std::size_t i = 0; i < mb.nrBodies(); ++i)
	{
		Vector3d comT = mb.body(i).inertia().momentum()/
			mb.body(i).inertia().mass();
		jacVec_[i] = Jacobian(mb, mb.body(i).id(), comT);
	}
}


CoMJacobianDummy::~CoMJacobianDummy()
{}


const Eigen::MatrixXd&
CoMJacobianDummy::jacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	jac_.setZero();

	for(std::size_t i = 0; i < mb.nrBodies(); ++i)
	{
		const MatrixXd& jac = jacVec_[i].jacobian(mb, mbc);
		jacVec_[i].fullJacobian(mb, jac, jacFull_);
		jac_ += jacFull_*bodies[i].inertia().mass();
	}

	jac_ /= mb.nrBodies();

	return jac_;
}


} // namespace rbd

