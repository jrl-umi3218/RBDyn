// Copyright 2012-2016 CNRS-UM LIRMM, CNRS-AIST JRL
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

#pragma once

// includes
// std
#include <map>
#include <memory>
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "Body.h"
#include "Joint.h"

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;

/**
	* Graph representation of the robot.
	* Provide a undirected graph representation of the robot that allow
	* to create a kinematic tree from any body as root.
	* The graph must be cycle free (closed loop is not supported).
	*/
class RBDYN_DLLAPI MultiBodyGraph
{
public:
	struct Node;

	/**
		*.Arc of the multibody graph.
		* Represent a joint of a body.
		*/
	struct Arc
	{
		Arc() {}

		Arc(sva::PTransformd X0, const Joint& j, bool forward, std::shared_ptr<Node> n):
			X(X0),
			joint(j),
			next(n)
		{
			joint.forward(forward);
		}

		sva::PTransformd X; ///< Position of the joint in body coordinate.
		Joint joint; ///< Joint with right direction.
		std::shared_ptr<Node> next; ///< successor node.
	};

	/**
		* Node of the multibody graph.
		* Represent the body.
		*/
	struct Node
	{
		Node(const Body& b):
			body(b)
		{}

		Body body;
		std::vector<Arc> arcs; ///< Outgoing arc.
	};

public:
	MultiBodyGraph();
	MultiBodyGraph(const MultiBodyGraph& mbg);
	~MultiBodyGraph();

	MultiBodyGraph& operator=(const MultiBodyGraph& mbg);

	void clear();

	/**
		* Add a node to the graph.
		* @param B Body to add, his body id must be unique.
		* @throw std::domain_error If the body id already exist or if body id
		* is less than zero.
		*/
	void addBody(const Body& B);

	/**
		* Add a joint.
		* @param J Joint to add, his joint id must be unique.
		* @throw std::domain_error If the joint id on name already exist or if joint
		* id is less than zero.
		*/
	void addJoint(const Joint& J);

	/**
		* Create an arc to the graph.
		* @param b1Id Body 1 id.
		* @param tB1 Transformation from body 1 to joint in body 1 coordinate.
		* @param b2Id Body 2 id.
		* @param tB2 Transformation from body 2 to joint in body 2 coordinate.
		* @param jointId Joint betweend the two body id.
		* @param isB1toB2 If true use the joint forward value as body 1 to body 2
		* forward value of the joint and the inverse value for body 2 to body 1
		* joint. If false the behavior is inversed.
		* @throw std::out_of_range If b1Id or b2Id or jointId don't exist.
		*/
	void linkBodies(int b1Id, const sva::PTransformd& tB1,
		int b2Id, const sva::PTransformd& tB2, int jointId, bool isB1toB2=true);

	/**
		* @param id Id of the node.
		* @return Node with the Id id.
		* @throw std::out_of_range if id don't exist.
		*/
	const std::shared_ptr<Node> nodeById(int id) const;

	/**
		* @param id Id of the joint.
		* @return Joint with the Id id.
		* @throw std::out_of_range If id don't exist.
		*/
	const std::shared_ptr<Joint> jointById(int id) const;

	/**
		* @param id Name of the joint.
		* @return Joint with the Name name.
		* @throw std::out_of_range If name don't exist.
		*/
	const std::shared_ptr<Joint> jointByName(const std::string& name) const;

	/**
		* @param name Name of the joint.
		* @return Id of the joint named name.
		* @throw std::out_of_range If joint named name don't exist.
		*/
	int jointIdByName(const std::string& name) const;

	/**
		* @param name Name of the body.
		* @return Id of the body named name.
		* @throw std::out_of_range If body named name don't exist.
		*/
	int bodyIdByName(const std::string& name) const;

	/// @return Numbers of nodes.
	std::size_t nrNodes() const;

	/// @return Numbers of joints.
	std::size_t nrJoints() const;

	/**
		* Create a MultiBody from the graph.
		* @param rootBodyId Id of the root body.
		* @param isFixed True if the root is fixed, false if the root is free.
		* @param X_0_j0 Transformation to the root joint.
		* @param X_b0_j0 Transformation from the root body to the root joint.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	MultiBody makeMultiBody(int rootBodyId, bool isFixed,
		const sva::PTransformd& X_0_j0=sva::PTransformd::Identity(),
		const sva::PTransformd& X_b0_j0=sva::PTransformd::Identity());

	/**
		* Create a MultiBody from the graph.
		* @param rootBodyId Id of the root body.
		* @param rootJointType MultiBody root joint type.
		* @param X_0_j0 Transformation to the root joint.
		* @param X_b0_j0 Transformation from the root body to the root joint.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	MultiBody makeMultiBody(int rootBodyId, Joint::Type rootJointType,
		const sva::PTransformd& X_0_j0=sva::PTransformd::Identity(),
		const sva::PTransformd& X_b0_j0=sva::PTransformd::Identity());

	/**
		* Create a MultiBody from the graph.
		* @param rootBodyId Id of the root body.
		* @param rootJointType MultiBody root joint type.
		* @param rootJointAxid MultiBody root joint axis.
		* @param X_0_j0 Transformation to the root joint.
		* @param X_b0_j0 Transformation from the root body to the root joint.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	MultiBody makeMultiBody(int rootBodyId, Joint::Type rootJointType,
		const Eigen::Vector3d& axis,
		const sva::PTransformd& X_0_j0=sva::PTransformd::Identity(),
		const sva::PTransformd& X_b0_j0=sva::PTransformd::Identity());

	/**
		* Remove a joint (and successor joints and bodies) from the graph.
		* @param rootBodyId Graph root.
		* @param jointId joint to remove.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	void removeJoint(int rootBodyId, int jointId);

	/**
		* Remove a joint (and successor joints and bodies) from the graph.
		* Do nothing if jointName doesn't existe.
		* @param rootBodyId Graph root.
		* @param jointName joint to remove.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	void removeJoint(int rootBodyId, const std::string& jointName);

	/**
		* Remove joints (and successor joints and bodies) from the graph.
		* @param rootBodyId Graph root.
		* @param joints List of joints id to remove.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	void removeJoints(int rootBodyId, const std::vector<int>& joints);

	/**
		* Remove joints (and successor joints and bodies) from the graph.
		* Do nothing for each joint name that doesn't exist.
		* @param rootBodyId Graph root.
		* @param joints List of joints name to remove.
		* @throw std::out_of_range If rootBodyId don't exist.
		*/
	void removeJoints(int rootBodyId, const std::vector<std::string>& joints);

	/**
		* Merge jointId child bodies in jointId parent body (merge inertia matrix)
		* @param rootBodyId Graph root.
		* @param jointId Merge all node after jointId in jointId parent node.
		* @param jointPosById Joint configuarition by joint id.
		* @throw std::out_of_range, std::domain_error.
		*/
	void mergeSubBodies(int rootBodyId, int jointId,
										 const std::map<int, std::vector<double>>& jointPosById);

	/**
		* Merge jointName child bodies in jointName parent body (merge inertia matrix)
		* @param rootBodyId Graph root.
		* @param jointName Merge all node after jointName in jointName parent node.
		* @param jointPosById Joint configuarition by joint id.
		* @throw std::out_of_range, std::domain_error.
		*/
	void mergeSubBodies(int rootBodyId, const std::string& jointName,
										 const std::map<int, std::vector<double>>& jointPosById);

	/**
		* Compute the transformation to apply on each bodies to find their original
		* base.
		* @param rootBodyId Graph root.
		* @param X_b0_j0 Transformation from the root body to the root joint.
		* @return Computed transformations by body id.
		* @throw std::out_of_range.
		*/
	std::map<int, sva::PTransformd> bodiesBaseTransform(int rootBodyId,
		const sva::PTransformd& X_b0_j0=sva::PTransformd::Identity());

	/**
		* Return the dictionary of successor joints id by body id.
		* @param rootBodyId Graph root.
		* @return dictionary of successor joints id by body id.
		* @throw std::out_of_range.
		*/
	std::map<int, std::vector<int>> successorJoints(int rootBodyId);

	/**
		* Return the dictionary of predecessor joint id by body id.
		* @param rootBodyId Graph root.
		* @return dictionary of predecessor joint id by body id.
		* @throw std::out_of_range.
		*/
	std::map<int, int> predecessorJoint(int rootBodyId);

private:
	/**
		* Find the arc jointId and remove it from the graph with his sub node.
		* This function is recursive.
		*/
	bool rmArc(Node& node, int parentJointId, int jointId);

	/**
		* Remove all joint with same id than Arc::joint from MultiBodyGraph and call
		* rmNodeFromMbg on Arc::next.
		*/
	void rmArcFromMbg(const Arc& arc);

	/**
		* Call rmArcFromMbg on all outgoing arc that don't have an id equal to
		* jointIdFrom and remove node from MultiBodyGraph
		*/
	void rmNodeFromMbg(int jointIdFrom, const std::shared_ptr<Node>& node);

	/**
		* Find the arc jointId, merge all jointId sub nodes in jointId parent
		* sub node and remove jointId and his sub nodes from mbg.
		* This function is recursive.
		*/
	bool findMergeSubNodes(Node& node, int parentJointId, int jointId,
		const std::map<int, std::vector<double>>& jointPosById);

	/**
		* Return node inertia merged with all his sub node in parentJointId
		* frame (after joint transform).
		*/
	sva::RBInertiad mergeSubNodes(Node& node, int parentJointId,
		const std::map<int, std::vector<double>>& jointPosById);

	/**
		* Merge parent and child inertia with parent in inertia in parent body base
		* and child inertia in joint base (after transform).
		*/
	sva::RBInertiad mergeInertia(const sva::RBInertiad& parentInertia,
		const sva::RBInertiad& childInertia, const Joint& joint,
		const sva::PTransformd& X_p_j,
		const std::map<int, std::vector<double>>& jointPosById);

	// copy mbg in this. this must be empty before calling this function.
	void copy(const rbd::MultiBodyGraph& mbg);


private:
	std::vector<std::shared_ptr<Node>> nodes_;
	std::vector<std::shared_ptr<Joint>> joints_;

	std::map<int, std::shared_ptr<Node>> bodyId2Node_;
	std::map<int, std::shared_ptr<Joint>> jointId2Joint_;

	std::map<std::string, int> jointName2Id_;
	std::map<std::string, int> bodyName2Id_;
};

}
