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
	// check that the joint id don't exist
	if(jointId2Joint_.find(J.id()) != jointId2Joint_.end())
	{
		std::ostringstream msg;
		msg << "Joint id: "  << J.id() << " already exist.";
		throw std::domain_error(msg.str());
	}

	// check that the joint name don't exist
	if(jointName2Id_.find(J.name()) != jointName2Id_.end())
	{
		std::ostringstream msg;
		msg << "Joint name: "  << J.name() << " already exist.";
		throw std::domain_error(msg.str());
	}

	joints_.push_back(std::make_shared<Joint>(J));
	jointId2Joint_[J.id()] = joints_.back();
	jointName2Id_[J.name()] = J.id();
}

void MultiBodyGraph::linkBodies(int b1Id, const sva::PTransformd& tB1,
	int b2Id, const sva::PTransformd& tB2, int jointId, bool isB1toB2)
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

const std::shared_ptr<Joint>
MultiBodyGraph::jointByName(const std::string& name) const
{
	return jointById(jointIdByName(name));
}

int MultiBodyGraph::jointIdByName(const std::string& name) const
{
	return jointName2Id_.at(name);
}

std::size_t MultiBodyGraph::nrNodes() const
{
	return nodes_.size();
}

std::size_t MultiBodyGraph::nrJoints() const
{
	return joints_.size();
}

MultiBody MultiBodyGraph::makeMultiBody(int rootBodyId, bool isFixed,
	const sva::PTransformd& initTrans)
{
	using namespace Eigen;

	std::vector<Body> bodies;
	std::vector<Joint> joints;

	std::vector<int> pred;
	std::vector<int> succ;
	std::vector<int> parent;
	std::vector<sva::PTransformd> Xt;

	std::shared_ptr<Node> rootNode = bodyId2Node_.at(rootBodyId);
	Joint rootJoint = isFixed ? Joint(Joint::Fixed, true, -1, "Root") :
		Joint(Joint::Free, true, -1, "Root");

	std::function<void(const std::shared_ptr<Node> curNode,
										 const std::shared_ptr<Node> fromNode, const Joint& joint,
										 int p, int s, int par,
										 const sva::PTransformd& Xt)> makeTree;

	makeTree = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode, const Joint& joint,
		int p, int s, int par,
		const sva::PTransformd& Xti)
	{
		// looking for transformation that come from fromNode
		sva::PTransformd XFrom = sva::PTransformd::Identity();
		for(Arc& a : curNode->arcs)
		{
			if(a.next == fromNode)
			{
				XFrom = a.X;
				break;
			}
		}

		bodies.emplace_back(XFrom.dualMul(curNode->body.inertia()),
			curNode->body.id(), curNode->body.name());
		joints.push_back(joint);
		pred.push_back(p);
		succ.push_back(s);
		parent.push_back(par);
		Xt.push_back(Xti);

		int curInd = static_cast<int>(bodies.size()) - 1;
		for(Arc& a : curNode->arcs)
		{
			if(a.next != fromNode)
			{
				int nextInd = static_cast<int>(bodies.size());

				makeTree(a.next, curNode, a.joint, curInd, nextInd, curInd,
					a.X*XFrom.inv());
			}
		}
	};

	makeTree(rootNode, nullptr, rootJoint, -1, 0, -1, initTrans);

	return MultiBody(std::move(bodies), std::move(joints),
		std::move(pred), std::move(succ), std::move(parent), std::move(Xt));
}

void MultiBodyGraph::removeJoint(int rootBodyId, int jointId)
{
	std::shared_ptr<Node> rootNode = bodyId2Node_.at(rootBodyId);

	// only destroy the joint if it exist
	if(bodyId2Node_.find(jointId) != bodyId2Node_.end())
	{
		rmArc(*rootNode, -1, jointId);
	}
}

void MultiBodyGraph::removeJoint(int rootBodyId, const std::string& jointName)
{
	// only destroy the joint if it exist
	const auto it = jointName2Id_.find(jointName);
	if(it != jointName2Id_.end())
	{
		removeJoint(rootBodyId, it->second);
	}
}

void MultiBodyGraph::removeJoints(int rootBodyId,
	const std::vector<int>& joints)
{
	std::shared_ptr<Node> rootNode = bodyId2Node_.at(rootBodyId);

	for(int id: joints)
	{
		// only destroy the joint if it exist
		if(bodyId2Node_.find(id) != bodyId2Node_.end())
		{
			rmArc(*rootNode, -1, id);
		}
	}
}

void MultiBodyGraph::removeJoints(int rootBodyId,
	const std::vector<std::string>& joints)
{
	std::shared_ptr<Node> rootNode = bodyId2Node_.at(rootBodyId);

	for(const std::string& name: joints)
	{
		// only destroy the joint if it exist
		const auto it = jointName2Id_.find(name);
		if(it != jointName2Id_.end())
		{
			rmArc(*rootNode, -1, it->second);
		}
	}
}

bool MultiBodyGraph::rmArc(Node& node, int parentJointId, int jointId)
{
	// depth first exploration of the graph
	// end when the joint is found
	for(auto it = node.arcs.begin(); it != node.arcs.end(); ++it)
	{
		if(it->joint.id() != parentJointId)
		{
			if(it->joint.id() == jointId)
			{
				rmArcFromMbg(*it);
				node.arcs.erase(it);
				return true;
			}
			else
			{
				if(rmArc(*it->next, it->joint.id(), jointId))
				{
					return true;
				}
			}
		}
	}
	return false;
}

void MultiBodyGraph::rmArcFromMbg(const Arc& arc)
{
	const std::shared_ptr<Joint>& joint = jointId2Joint_.at(arc.joint.id());
	// erase the joint from joints list
	joints_.erase(std::find(joints_.begin(), joints_.end(), joint));
	// erase from map
	jointId2Joint_.erase(arc.joint.id());
	jointName2Id_.erase(arc.joint.name());
	// rm the sub node from joint list
	rmNodeFromMbg(arc.joint.id(), arc.next);
}

void MultiBodyGraph::rmNodeFromMbg(int jointIdFrom,
																const std::shared_ptr<Node>& node)
{
	for(const Arc& arc: node->arcs)
	{
		// if we call rmArcFromMbg on the parent joint a mass destruction will
		// occur
		if(arc.joint.id() != jointIdFrom)
		{
			rmArcFromMbg(arc);
		}
	}
	node->arcs.clear();

	bodyId2Node_.erase(node->body.id());
	nodes_.erase(std::find(nodes_.begin(), nodes_.end(), node));
}

} // namespace rbd
