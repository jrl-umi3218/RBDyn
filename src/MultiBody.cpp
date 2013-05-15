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
#include "MultiBody.h"

// includes
// RBDyn
#include "Body.h"
#include "Joint.h"

namespace rbd
{

MultiBody::MultiBody():
  nrParams_(0),
  nrDof_(0)
{}

MultiBody::MultiBody(std::vector<Body> bodies, std::vector<Joint> joints,
	std::vector<int> pred, std::vector<int> succ,
	std::vector<int> parent,
	std::vector<sva::PTransform> Xto):
	bodies_(std::move(bodies)),
	joints_(std::move(joints)),
	pred_(std::move(pred)),
	succ_(std::move(succ)),
	parent_(std::move(parent)),
	Xt_(std::move(Xto)),
	jointPosInParam_(joints_.size()),
	jointPosInDof_(joints_.size()),
	nrParams_(0),
	nrDof_(0)
{
	for(int i = 0; i < static_cast<int>(bodies_.size()); ++i)
	{
		bodyId2Ind_[bodies_[i].id()] = i;
		jointId2Ind_[joints_[i].id()] = i;

		jointPosInParam_[i] = nrParams_;
		jointPosInDof_[i] = nrDof_;

		nrParams_ += joints_[i].params();
		nrDof_ += joints_[i].dof();
	}
}

} // namespace rbd
