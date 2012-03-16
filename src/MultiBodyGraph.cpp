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
#include "MultiBodyGraph.h"

// includes
// RBDyn
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"

namespace rbd
{

MultiBodyGraph::Node::Node(const Body& b):
	body(std::make_shared<Body>(b))
{
}

void MultiBodyGraph::addBody(const Body& B)
{
	nodes_.push_back(std::make_shared<Node>(B));
	bodyId2Node_[B.id()] = nodes_.back();
}

void MultiBodyGraph::addJoint(const Joint& J)
{
	joints_.push_back(std::make_shared<Joint>(J));
	jointId2Joint_[J.id()] = joints_.back();
}

void MultiBodyGraph::linkBodies(int b1Id, const sva::PTransform& tB1,
	int b2Id, const sva::PTransform& tB2, int jointId)
{
	std::shared_ptr<Node> b1 = bodyId2Node_.at(b1Id);
	std::shared_ptr<Node> b2 = bodyId2Node_.at(b2Id);
	std::shared_ptr<Joint> j = jointId2Joint_.at(jointId);

	b1->arcs.emplace_back(tB1, j, b2);
	b2->arcs.emplace_back(tB2, j, b1);
}

MultiBody MultiBodyGraph::makeMultibBody(int rootBodyId, bool isFixed)
{
	using namespace Eigen;

	std::vector<Body> bodies;
	std::vector<Joint> joints;

	std::vector<int> pred;
	std::vector<int> succ;
	std::vector<int> parent;
	std::vector<sva::PTransform> Xt;

	std::shared_ptr<Node> rootNode = bodyId2Node_[rootBodyId];
	Joint rootJoint = isFixed ? Joint(Joint::Fixed, -1, "Root") :
		Joint(Joint::Free, -1, "Root");

	std::function<void(const std::shared_ptr<Node> curNode,
										 const std::shared_ptr<Node> fromNode, const Joint& joint,
										 int p, int s, int par,
										 const sva::PTransform& X)> makeTree;

	makeTree = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode, const Joint& joint,
		int p, int s, int par,
		const sva::PTransform& X)
	{
		bodies.push_back(*(curNode->body));
		joints.push_back(joint);
		pred.push_back(p);
		succ.push_back(s);
		parent.push_back(par);
		Xt.push_back(X);

		int curInd = bodies.size() - 1;

		sva::PTransform fromX;
		for(Arc& a : curNode->arcs)
		{
			if(a.next == fromNode)
			{
				fromX =  a.X.inv();
				break;
			}
		}

		for(Arc& a : curNode->arcs)
		{
			if(a.next != fromNode)
			{
				makeTree(a.next, curNode, *a.joint, curInd, curInd + 1, curInd, a.X*fromX);
			}
		}
	};

	makeTree(rootNode, nullptr, rootJoint, -1, 0, -1, sva::PTransform::Identity());

	return MultiBody(std::move(bodies), std::move(joints),
		std::move(pred), std::move(succ), std::move(parent), std::move(Xt));
}

} // namespace rbd
