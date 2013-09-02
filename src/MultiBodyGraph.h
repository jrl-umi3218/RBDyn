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

namespace rbd
{
class MultiBody;

/**
	* Graph representation of the robot.
	* Provide a undirected graph representation of the robot that allow
	* to create a kinematic tree from any body as root.
	* The graph must be cycle free (closed loop is not supported).
	*/
class MultiBodyGraph
{
public:
	struct Node;

	/**
		*.Arc of the multibody graph.
		* Represent a joint of a body.
		*/
	struct Arc
	{
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

	/**
		* Add a node to the graph.
		* @param B Body to add, his body id must be unique.
		* @throw std::domain_error If the body id already exist.
		*/
	void addBody(const Body& B);

	/**
		* Add a joint.
		* @param J Joint to add, his joint id must be unique.
		* @throw std::domain_error If the joint id already exist.
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

	/// @return Numbers of nodes.
	std::size_t nrNodes() const;

	/// @return Numbers of joints.
	std::size_t nrJoints() const;

	/**
		* Create a MultiBody from the graph.
		* @param rootBodyId Id of the root body.
		* @param isFixed True if the root is fixed, false if the root is free.
		* @param initTrans Initial transformation of the robot root.
		* @throw std::out_of_rang If rootBodyId don't exist.
		*/
	MultiBody makeMultiBody(int rootBodyId, bool isFixed,
		const sva::PTransformd& initTrans=sva::PTransformd::Identity());

private:
	std::vector<std::shared_ptr<Node>> nodes_;
	std::vector<std::shared_ptr<Joint>> joints_;

	std::map<int, std::shared_ptr<Node>> bodyId2Node_;
	std::map<int, std::shared_ptr<Joint>> jointId2Joint_;
};

}
