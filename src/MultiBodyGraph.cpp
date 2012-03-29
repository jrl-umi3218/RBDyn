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
// std
#include <sstream>
#include <stdexcept>

// RBDyn
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"

namespace rbd
{

void MultiBodyGraph::addBody(const Body& B)
{
	if(bodyId2Node_.find(B.id()) != bodyId2Node_.end())
	{
		std::ostringstream msg;
		msg << "Body id: "  << B.id() << " already exist.";
		throw std::domain_error(msg.str());
	}
	nodes_.push_back(std::make_shared<Node>(B));
	bodyId2Node_[B.id()] = nodes_.back();
}

void MultiBodyGraph::addJoint(const Joint& J)
{
	if(jointId2Joint_.find(J.id()) != jointId2Joint_.end())
	{
		std::ostringstream msg;
		msg << "Joint id: "  << J.id() << " already exist.";
		throw std::domain_error(msg.str());
	}
	joints_.push_back(std::make_shared<Joint>(J));
	jointId2Joint_[J.id()] = joints_.back();
}

void MultiBodyGraph::linkBodies(int b1Id, const sva::PTransform& tB1,
	int b2Id, const sva::PTransform& tB2, int jointId, bool isB1toB2)
{
	std::shared_ptr<Node> b1 = bodyId2Node_.at(b1Id);
	std::shared_ptr<Node> b2 = bodyId2Node_.at(b2Id);
	std::shared_ptr<Joint> j = jointId2Joint_.at(jointId);

	bool fj1 = isB1toB2 ? j->forward() : !j->forward();
	bool fj2 = !fj1;

	b1->arcs.emplace_back(tB1, *j, fj1, b2);
	b2->arcs.emplace_back(tB2, *j, fj2, b1);
}

const std::shared_ptr<MultiBodyGraph::Node> MultiBodyGraph::nodeById(int id) const
{
	return bodyId2Node_.at(id);
}

const std::shared_ptr<Joint> MultiBodyGraph::jointById(int id) const
{
	return jointId2Joint_.at(id);
}

std::size_t MultiBodyGraph::nrNodes() const
{
	return nodes_.size();
}

std::size_t MultiBodyGraph::nrJoints() const
{
	return joints_.size();
}

MultiBody MultiBodyGraph::makeMultiBody(int rootBodyId, bool isFixed)
{
	using namespace Eigen;

	std::vector<Body> bodies;
	std::vector<Joint> joints;

	std::vector<int> pred;
	std::vector<int> succ;
	std::vector<int> parent;
	std::vector<sva::PTransform> Xfrom;
	std::vector<sva::PTransform> Xto;

	std::shared_ptr<Node> rootNode = bodyId2Node_.at(rootBodyId);
	Joint rootJoint = isFixed ? Joint(Joint::Fixed, true, -1, "Root") :
		Joint(Joint::Free, true, -1, "Root");

	std::function<void(const std::shared_ptr<Node> curNode,
										 const std::shared_ptr<Node> fromNode, const Joint& joint,
										 int p, int s, int par,
										 const sva::PTransform& Xfrom,
										 const sva::PTransform& Xto)> makeTree;

	makeTree = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode, const Joint& joint,
		int p, int s, int par,
		const sva::PTransform& Xf,
		const sva::PTransform& Xt)
	{
		bodies.push_back(curNode->body);
		joints.push_back(joint);
		pred.push_back(p);
		succ.push_back(s);
		parent.push_back(par);
		Xfrom.push_back(Xf);
		Xto.push_back(Xt);

		// looking for transformation that come from fromNode

		int curInd = bodies.size() - 1;
		for(Arc& a : curNode->arcs)
		{
			if(a.next != fromNode)
			{
				int nextInd = bodies.size();

				sva::PTransform toSuccCenter;
				for(Arc& aN : a.next->arcs)
				{
					if(aN.next == curNode)
					{
						toSuccCenter = aN.X.inv();
						break;
					}
				}

				makeTree(a.next, curNode, a.joint, curInd, nextInd, curInd,
					a.X, toSuccCenter);
			}
		}
	};

	makeTree(rootNode, nullptr, rootJoint, -1, 0, -1,
		sva::PTransform::Identity(), sva::PTransform::Identity());

	return MultiBody(std::move(bodies), std::move(joints),
		std::move(pred), std::move(succ), std::move(parent), std::move(Xfrom),
		std::move(Xto));
}

} // namespace rbd
