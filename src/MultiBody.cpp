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

// associated header
#include "RBDyn/MultiBody.h"

// includes
// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/Joint.h"

namespace rbd
{

MultiBody::MultiBody():
  nrParams_(0),
  nrDof_(0)
{}

MultiBody::MultiBody(std::vector<Body> bodies, std::vector<Joint> joints,
	std::vector<int> pred, std::vector<int> succ,
	std::vector<int> parent,
	std::vector<sva::PTransformd> Xto):
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
		bodyNameToInd_[bodies_[i].name()] = i;
		jointNameToInd_[joints_[i].name()] = i;

		jointPosInParam_[i] = nrParams_;
		jointPosInDof_[i] = nrDof_;

		nrParams_ += joints_[i].params();
		nrDof_ += joints_[i].dof();
	}
}

} // namespace rbd
