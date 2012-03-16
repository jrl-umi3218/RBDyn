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
#include <stdexcept>
#include <unordered_map>
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg>

namespace rbd
{
class Body;
class Joint;

class MultiBody
{
public:
	MultiBody(std::vector<Body> bodies, std::vector<Joint> joints,
		std::vector<int> pred, std::vector<int> succ,
		std::vector<int> parent, std::vector<sva::PTransform> Xt);

	std::size_t nrBodies() const
	{
		return bodies_.size();
	}

	std::size_t nrJoints() const
	{
		return joints_.size();
	}

	const std::vector<Body>& bodies() const
	{
		return bodies_;
	}

	const Body& body(int num) const
	{
		return bodies_[num];
	}

	const std::vector<Joint>& joints() const
	{
		return joints_;
	}

	const Joint& joint(int num) const
	{
		return joints_[num];
	}

	const std::vector<int>& predecessors() const
	{
		return pred_;
	}

	int predecessor(int num) const
	{
		return pred_[num];
	}

	const std::vector<int>& successors() const
	{
		return succ_;
	}

	int successor(int num) const
	{
		return succ_[num];
	}

	const std::vector<int>& parents() const
	{
		return parent_;
	}

	int parent(int num) const
	{
		return parent_[num];
	}

	const std::vector<sva::PTransform>& transforms() const
	{
		return Xt_;
	}

	const sva::PTransform& transform(int num) const
	{
		return Xt_[num];
	}

	int bodyIndexById(int id) const
	{
		return bodyId2Ind_.find(id)->second;
	}

	int jointIndexById(int id) const
	{
		return jointId2Ind_.find(id)->second;
	}


	// safe accessors version for python binding

	const Body& sBody(int num) const
	{
		return bodies_.at(num);
	}

	const Joint& sJoint(int num) const
	{
		return joints_.at(num);
	}

	int sPredecessor(int num) const
	{
		return pred_.at(num);
	}

	int sSuccessor(int num) const
	{
		return succ_.at(num);
	}

	int sParent(int num) const
	{
		return parent_.at(num);
	}

	const sva::PTransform& sTransform(int num) const
	{
		return Xt_.at(num);
	}

	int sBodyIndexById(int id) const
	{
		return bodyId2Ind_.at(id);
	}

	int sJointIndexById(int id) const
	{
		return jointId2Ind_.at(id);
	}

private:
	std::vector<Body> bodies_;
	std::vector<Joint> joints_;

	std::vector<int> pred_;
	std::vector<int> succ_;
	std::vector<int> parent_;
	std::vector<sva::PTransform> Xt_;

	std::unordered_map<int, int> bodyId2Ind_;
	std::unordered_map<int, int> jointId2Ind_;
};

} // namespace rbd
