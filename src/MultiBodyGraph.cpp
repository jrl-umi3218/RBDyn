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
#include "RBDyn/MultiBodyGraph.h"

// includes
// std
#include <sstream>
#include <stdexcept>

// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"

namespace rbd
{

MultiBodyGraph::MultiBodyGraph():
	nodes_(),
	joints_(),
	bodyNameToNode_(),
	jointNameToJoint_()
{
}

MultiBodyGraph::MultiBodyGraph(const MultiBodyGraph& mbg)
{
	copy(mbg);
}

MultiBodyGraph::~MultiBodyGraph()
{
	clear();
}

MultiBodyGraph& MultiBodyGraph::operator=(const MultiBodyGraph& mbg)
{
	if(&mbg != this)
	{
		clear();
		copy(mbg);
	}

	return *this;
}

void MultiBodyGraph::clear()
{
	// destroy all arc to avoid cyclic reference on nodes
	for(const std::shared_ptr<Node>& node: nodes_)
	{
		node->arcs.clear();
	}

	nodes_.clear();
	joints_.clear();
	bodyNameToNode_.clear();
	jointNameToJoint_.clear();
}

void MultiBodyGraph::addBody(const Body& B)
{

	if(bodyNameToNode_.find(B.name()) != bodyNameToNode_.end())
	{
		std::ostringstream msg;
		msg << "Body name: "  << B.name() << " already exists.";
		throw std::domain_error(msg.str());
	}

	nodes_.push_back(std::make_shared<Node>(B));
	bodyNameToNode_[B.name()] = nodes_.back();
}

void MultiBodyGraph::addJoint(const Joint& J)
{
	// check that the joint name don't exist
	if(jointNameToJoint_.find(J.name()) != jointNameToJoint_.end())
	{
		std::ostringstream msg;
		msg << "Joint name: "  << J.name() << " already exists.";
		throw std::domain_error(msg.str());
	}

	joints_.push_back(std::make_shared<Joint>(J));
	jointNameToJoint_[J.name()] = joints_.back();
}

void MultiBodyGraph::linkBodies(const std::string& b1Name, const sva::PTransformd& tB1,
				const std::string& b2Name, const sva::PTransformd& tB2,
				const std::string& jointName, bool isB1toB2)
{
	std::shared_ptr<Node> b1 = bodyNameToNode_.at(b1Name);
	std::shared_ptr<Node> b2 = bodyNameToNode_.at(b2Name);
	std::shared_ptr<Joint> j = jointNameToJoint_.at(jointName);

	bool fj1 = isB1toB2 ? j->forward() : !j->forward();
	bool fj2 = !fj1;

	b1->arcs.emplace_back(tB1, *j, fj1, b2);
	b2->arcs.emplace_back(tB2, *j, fj2, b1);
}

const std::shared_ptr<MultiBodyGraph::Node>
MultiBodyGraph::nodeByName(const std::string& name) const
{
	return bodyNameToNode_.at(name);
}

const std::shared_ptr<Joint>
MultiBodyGraph::jointByName(const std::string& name) const
{
	return jointNameToJoint_.at(name);
}

std::size_t MultiBodyGraph::nrNodes() const
{
	return nodes_.size();
}

std::size_t MultiBodyGraph::nrJoints() const
{
	return joints_.size();
}

MultiBody MultiBodyGraph::makeMultiBody(const std::string& rootBodyName, bool isFixed,
	const sva::PTransformd& X_0_j0, const sva::PTransformd& X_b0_j0)
{
	return makeMultiBody(rootBodyName, isFixed ? Joint::Fixed : Joint::Free,
		X_0_j0, X_b0_j0);
}

MultiBody MultiBodyGraph::makeMultiBody(const std::string& rootBodyName,
	Joint::Type rootJointType, const sva::PTransformd& X_0_j0,
	const sva::PTransformd& X_b0_j0)
{
	return makeMultiBody(rootBodyName, rootJointType, Eigen::Vector3d::UnitZ(),
		X_0_j0, X_b0_j0);
}

MultiBody MultiBodyGraph::makeMultiBody(const std::string& rootBodyName,
	Joint::Type rootJointType, const Eigen::Vector3d& axis,
	const sva::PTransformd& X_0_j0, const sva::PTransformd& X_b0_j0)
{
	using namespace Eigen;

	std::vector<Body> bodies;
	std::vector<Joint> joints;

	std::vector<int> pred;
	std::vector<int> succ;
	std::vector<int> parent;
	std::vector<sva::PTransformd> Xt;

	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);
	Joint rootJoint(rootJointType, axis, true, "Root");

	std::function<void(const std::shared_ptr<Node> curNode,
			const std::shared_ptr<Node> fromNode,
			const Joint& joint,
			int p, int s, int par,
			const sva::PTransformd& Xt,
			const sva::PTransformd& Xbase)> makeTree;

	makeTree = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode, const Joint& joint,
		int p, int s, int par,
		const sva::PTransformd& Xti, const sva::PTransformd& Xbase)
	{
		// looking for transformation that go to fromNode
		sva::PTransformd XFrom = Xbase;
		for(Arc& a : curNode->arcs)
		{
			if(a.next == fromNode)
			{
				XFrom = a.X;
				break;
			}
		}

		bodies.emplace_back(XFrom.dualMul(curNode->body.inertia()),
			curNode->body.name());
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
					a.X*XFrom.inv(), sva::PTransformd::Identity());
			}
		}
	};

	makeTree(rootNode, nullptr, rootJoint, -1, 0, -1, X_0_j0, X_b0_j0);

	return MultiBody(std::move(bodies), std::move(joints),
		std::move(pred), std::move(succ), std::move(parent), std::move(Xt));
}

void MultiBodyGraph::removeJoint(const std::string& rootBodyName,
				const std::string& jointName)
{
	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);

	// only destroy the joint if it exists
	if(jointNameToJoint_.find(jointName) != jointNameToJoint_.end())
	{
		rmArc(*rootNode, "Root", jointName);
	}
}

void MultiBodyGraph::removeJoints(const std::string& rootBodyName,
	const std::vector<std::string>& joints)
{
	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);

	for(const std::string& name: joints)
	{
		// only destroy the joint if it exists
		if(jointNameToJoint_.find(name) != jointNameToJoint_.end())
		{
			removeJoint(rootBodyName, name);
		}
	}
}

void MultiBodyGraph::mergeSubBodies(const std::string& rootBodyName,
	const std::string& jointName,
	const std::map<std::string, std::vector<double>>& jointPosByName)
{
	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);
	findMergeSubNodes(*rootNode, "Root", jointName, jointPosByName);
}

std::map<std::string, sva::PTransformd>
MultiBodyGraph::bodiesBaseTransform(const std::string& rootBodyName,
				const sva::PTransformd& X_b0_j0)
{
	std::map<std::string, sva::PTransformd> X_nb_b;

	std::function<void(const std::shared_ptr<Node> curNode,
			const std::shared_ptr<Node> fromNode,
			const sva::PTransformd& Xbase)> computeTransform;

	computeTransform = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode, const sva::PTransformd& Xbase)
	{
		// looking for transformation that go to fromNode
		sva::PTransformd XFrom = Xbase;
		for(Arc& a : curNode->arcs)
		{
			if(a.next == fromNode)
			{
				XFrom = a.X;
				break;
			}
		}
		X_nb_b[curNode->body.name()] = XFrom.inv();

		for(Arc& a : curNode->arcs)
		{
			if(a.next != fromNode)
			{
				computeTransform(a.next, curNode, sva::PTransformd::Identity());
			}
		}
	};

	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);
	computeTransform(rootNode, nullptr, X_b0_j0);
	return std::move(X_nb_b);
}

std::map<std::string, std::vector<std::string>>
MultiBodyGraph::successorJoints(const std::string& rootBodyName)
{
	std::map<std::string, std::vector<std::string>> successorJoints;

	std::function<void(const std::shared_ptr<Node> curNode,
				const std::shared_ptr<Node> fromNode)> computeSuccesors;

	computeSuccesors = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode)
	{
		successorJoints[curNode->body.name()] = {};
		for(Arc& a : curNode->arcs)
		{
			if(a.next != fromNode)
			{
				successorJoints[curNode->body.name()].push_back(a.joint.name());
				computeSuccesors(a.next, curNode);
			}
		}
	};

	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);
	computeSuccesors(rootNode, nullptr);
	return std::move(successorJoints);
}

std::map<std::string, std::string>
MultiBodyGraph::predecessorJoint(const std::string& rootBodyName)
{
	std::map<std::string, std::string> predJoint;

	std::function<void(const std::shared_ptr<Node> curNode,
				const std::shared_ptr<Node> fromNode,
				const std::string& predJointName)> computePredecessor;

	computePredecessor = [&](const std::shared_ptr<Node> curNode,
		const std::shared_ptr<Node> fromNode,
		const std::string& predJointName)
	{
		predJoint[curNode->body.name()] = predJointName;

		for(Arc& a : curNode->arcs)
		{
			if(a.next != fromNode)
			{
				computePredecessor(a.next, curNode, a.joint.name());
			}
		}
	};

	std::shared_ptr<Node> rootNode = bodyNameToNode_.at(rootBodyName);
	computePredecessor(rootNode, nullptr, "Root");
	return std::move(predJoint);
}

MultiBodyGraph MultiBodyGraph::fixJoints(const MultiBodyGraph& other, 
        const std::vector<std::string>& jointsToFix, bool fixAllJoints)
{
  MultiBodyGraph mbg(other);
  if (fixAllJoints)
  {
    for (auto& jointPtr : mbg.joints_)
    {
      if (jointPtr != nullptr)
      {
        jointPtr.reset(new Joint(Joint::Type::Fixed, Eigen::Vector3d::Zero(), 
                       jointPtr->forward(), jointPtr->name()));
      }
    }
  }
  else
  {
    for (const std::string& jointName : jointsToFix)
    {
      auto jointPtr = mbg.jointNameToJoint_.at(jointName);
      jointPtr.reset(new Joint(Joint::Type::Fixed, Eigen::Vector3d::Zero(), 
                      jointPtr->forward(), jointPtr->name()));
    }
  }

  for (auto& node : mbg.nodes_)
  {
    if (node == nullptr)
    {
      continue;
    }
    for (auto& arc : node->arcs)
    {
      const auto & jName = arc.joint.name();
      for (const auto & n : jointsToFix)
      {
        if (n == jName || fixAllJoints)
        {
          arc.joint = Joint(Joint::Type::Fixed, Eigen::Vector3d::Zero(), 
                            arc.joint.forward(), arc.joint.name());
          break;
        }
      }
    }
  }
  return mbg;
}

bool MultiBodyGraph::rmArc(Node& node, const std::string& parentJointName,
		const std::string& jointName)
{
	// depth first exploration of the graph
	// end when the joint is found
	for(auto it = node.arcs.begin(); it != node.arcs.end(); ++it)
	{
		if(it->joint.name() != parentJointName)
		{
			if(it->joint.name() == jointName)
			{
				rmArcFromMbg(*it);
				node.arcs.erase(it);
				return true;
			}
			else
			{
				if(rmArc(*it->next, it->joint.name(), jointName))
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
	const std::shared_ptr<Joint>& joint = jointNameToJoint_.at(arc.joint.name());
	// erase the joint from joints list
	joints_.erase(std::find(joints_.begin(), joints_.end(), joint));
	// erase from map
	jointNameToJoint_.erase(arc.joint.name());
	// rm the sub node from joint list
	rmNodeFromMbg(arc.joint.name(), arc.next);
}

void MultiBodyGraph::rmNodeFromMbg(const std::string& jointNameFrom,
		const std::shared_ptr<Node>& node)
{
	for(const Arc& arc: node->arcs)
	{
		// if we call rmArcFromMbg on the parent joint a mass destruction will
		// occur
		if(arc.joint.name() != jointNameFrom)
		{
			rmArcFromMbg(arc);
		}
	}
	node->arcs.clear();

	bodyNameToNode_.erase(node->body.name());
	nodes_.erase(std::find(nodes_.begin(), nodes_.end(), node));
}

bool MultiBodyGraph::findMergeSubNodes(Node& node,
	const std::string& parentJointName, const std::string& jointName,
	const std::map<std::string, std::vector<double>>& jointPosByName)
{
	for(auto it = node.arcs.begin(); it != node.arcs.end(); ++it)
	{
		if(it->joint.name() != parentJointName)
		{
			if(it->joint.name() == jointName)
			{
				// compute the body inertia merged with childs of jointId
				sva::RBInertiad newInertia = mergeInertia(node.body.inertia(),
					mergeSubNodes(*it->next, jointName, jointPosByName),
					it->joint, it->X, jointPosByName);

				// create the new body with merged sub nodes
				node.body = Body(newInertia, node.body.name());

				// remove sub nodes
				rmArcFromMbg(*it);
				node.arcs.erase(it);
				return true;
			}
			else
			{
				if(findMergeSubNodes(*it->next, it->joint.name(),
							jointName, jointPosByName))
				{
					return true;
				}
			}
		}
	}
	return false;
}

sva::RBInertiad MultiBodyGraph::mergeSubNodes(Node& node,
	const std::string& parentJointName,
	const std::map<std::string, std::vector<double>>& jointPosByName)
{
	sva::RBInertiad newInertia(node.body.inertia());

	for(const Arc& arc: node.arcs)
	{
		if(arc.joint.name() != parentJointName)
		{
			newInertia = mergeInertia(newInertia,
				mergeSubNodes(*arc.next, arc.joint.name(), jointPosByName),
				arc.joint, arc.X, jointPosByName);
		}
	}

	// looking for transformation that go to parent joint
	sva::PTransformd X_cb_jp = sva::PTransformd::Identity();
	for(const Arc& arc : node.arcs)
	{
		if(arc.joint.name() == parentJointName)
		{
			X_cb_jp = arc.X;
			break;
		}
	}

	return X_cb_jp.dualMul(newInertia);
}

sva::RBInertiad MultiBodyGraph::mergeInertia(const sva::RBInertiad& parentInertia,
	const sva::RBInertiad& childInertia, const Joint& joint,
	const sva::PTransformd& X_p_j,
	const std::map<std::string, std::vector<double>>& jointPosByName)
{
	if(jointPosByName.find(joint.name()) == jointPosByName.end())
	{
		std::ostringstream msg;
		msg << "jointPosByName  must contain joint " <<
					 joint.name() << " configuration";
		throw std::out_of_range(msg.str());
	}

	if(int(jointPosByName.at(joint.name()).size()) != joint.params())
	{
		std::ostringstream msg;
		msg << "joint " << joint.name() << " needs " << joint.params() <<
					 " parameters";
		throw std::domain_error(msg.str());
	}

	sva::PTransformd jointConfig = joint.pose(jointPosByName.at(joint.name()));
	// transformation from current body to joint in next body
	sva::PTransformd X_cb_jnb = jointConfig*X_p_j;

	// set merged sub inertia in current body base and add it to the current inertia
	return parentInertia + X_cb_jnb.transMul(childInertia);
}

void MultiBodyGraph::copy(const rbd::MultiBodyGraph& mbg)
{
	// copy nodes (whitout Arc) en fill bodyId2Node
	for(const std::shared_ptr<Node>& node: mbg.nodes_)
	{
		Node newNode(node->body);
		nodes_.push_back(std::make_shared<Node>(newNode));
		bodyNameToNode_[node->body.name()] = nodes_.back();
	}

	// copy joints en fill jointId2Node
	for(const std::shared_ptr<Joint>& joint: mbg.joints_)
	{
		joints_.push_back(std::make_shared<Joint>(*joint));
		jointNameToJoint_[joint->name()] = joints_.back();
	}

	// create arc for each node
	for(std::size_t i = 0; i < nodes_.size(); ++i)
	{
		const Node& nodeToCopy = *mbg.nodes_[i];
		Node& nodeToFill = *nodes_[i];
		for(const Arc& arc: nodeToCopy.arcs)
		{
			Arc newArc;
			newArc.X = arc.X;
			newArc.joint = arc.joint;
			newArc.next = bodyNameToNode_[arc.next->body.name()];
			nodeToFill.arcs.push_back(newArc);
		}
	}
}

} // namespace rbd
