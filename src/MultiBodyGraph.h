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
#include <map>
#include <memory>
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg>

namespace rbd
{
class Body;
class Joint;
class MultiBody;

class MultiBodyGraph
{
public:
	struct Node;
	struct Arc
	{
		Arc(sva::PTransform X0, std::shared_ptr<Joint> j, std::shared_ptr<Node> n):
			X(X0),
			joint(j),
			next(n)
		{}

		sva::PTransform X;
		std::shared_ptr<Joint> joint;
		std::shared_ptr<Node> next;
	};

	struct Node
	{
		Node(const Body& body);

		std::shared_ptr<Body> body;
		std::vector<Arc> arcs;
	};

public:

	void addBody(const Body& B);
	void addJoint(const Joint& B);
	void linkBodies(int b1Id, const sva::PTransform& tB1,
		int b2Id, const sva::PTransform& tB2, int jointId);

	const std::shared_ptr<Node> nodeById(int id) const;
	const std::shared_ptr<Joint> jointById(int id) const;

	std::size_t nrNodes() const;
	std::size_t nrJoints() const;

	MultiBody makeMultiBody(int rootBodyId, bool isFixed);

private:
	std::vector<std::shared_ptr<Node>> nodes_;
	std::vector<std::shared_ptr<Joint>> joints_;

	std::map<int, std::shared_ptr<Node>> bodyId2Node_;
	std::map<int, std::shared_ptr<Joint>> jointId2Joint_;
};

}
