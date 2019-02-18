# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of RBDyn.
#
# RBDyn is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RBDyn is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

from eigen.c_eigen cimport *
from sva.c_sva cimport *
from libcpp.map cimport map
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool

cdef extern from "<RBDyn/Body.h>" namespace "rbd":
  cdef cppclass Body:
    Body()
    Body(const Body &)
    Body(const RBInertiad&, string)
    Body(double, const Vector3d&, const Matrix3d&, string)

    string name() const
    RBInertiad inertia() const

    bool operator==(const Body&)
    bool operator!=(const Body&)

cdef extern from "<RBDyn/Joint.h>" namespace "rbd":
  ctypedef enum JointType "rbd::Joint::Type":
    JointRev "rbd::Joint::Rev"
    JointPrism "rbd::Joint::Prism"
    JointSpherical "rbd::Joint::Spherical"
    JointPlanar "rbd::Joint::Planar"
    JointCylindrical "rbd::Joint::Cylindrical"
    JointFree "rbd::Joint::Free"
    JointFixed "rbd::Joint::Fixed"
  cdef cppclass Joint:
    Joint()
    Joint(const Joint &)
    Joint(JointType, const Vector3d&, bool, string)
    Joint(JointType, bool, string)

    JointType type() const
    double direction() const
    bool forward() const
    void forward(double)
    int params() const
    int dof() const
    string name() const
    MatrixXd motionSubspace() const

    #FIXME In Cython > 0.20 it is possible to specify a python exception to map all C++ exception to this one
    #PTransformd sPose(const vector[double]&) except +MemoryError
    #MotionVecd sMotion(const vector[double]&) except +MemoryError
    PTransformd sPose(const vector[double]&) except +
    MotionVecd sMotion(const vector[double]&) except +

    vector[double] zeroParam() const
    vector[double] zeroDof() const

    bool operator==(const Joint&)
    bool operator!=(const Joint&)

cdef extern from "<RBDyn/MultiBody.h>" namespace "rbd":
  cdef cppclass MultiBody:
    MultiBody()
    MultiBody(const MultiBody&)
    MultiBody(const vector[Body]&, const vector[Joint]&, const vector[int]&, const vector[int]&, const vector[int]&, const vector[PTransformd]&)

    int nrBodies() const
    int nrJoints() const
    int nrParams() const
    int nrDof() const

    vector[Body] bodies() const
    Body sBody(int) except +#OutOfRange #FIXME Should be const

    void sBodies(const vector[Body]&) except +#RuntimeError
    void sBody(int, const Body &) except +#OutOfRangeo

    vector[Joint] joints() const
    Joint sJoint(int) except +#Out of range

    vector[int] predecessors() const
    int sPredecessor(int) except +#OutOfRange

    vector[int] successors() const
    int sSuccessor(int) except +#OutOfRange

    vector[int] parents() const
    int sParent(int) except +#OutOfRange

    vector[PTransformd] transforms() const
    PTransformd sTransform(int) except +#OutOfRange

    void sTransforms(const vector[PTransformd]&) except +#RuntimeError
    void sTransform(int,const PTransformd&) except +#OutOfRange

    vector[int] jointsPosInParam() const
    int sJointPosInParam(int) except + #OutOfRange

    vector[int] jointsPosInDof() const
    int sJointPosInDof(int) except + #OutOfRange

    int sBodyIndexByName(const string& name) except + #OutOfRange
    int sJointIndexByName(const string& name) except + #OutOfRange

cdef extern from "<RBDyn/MultiBodyGraph.h>" namespace "rbd":
  cdef cppclass MultiBodyGraph:
    MultiBodyGraph()
    MultiBodyGraph(const MultiBodyGraph &)
    void addBody(const Body &) except +#ValueError
    void addJoint(const Joint &) except +#ValueError
    void linkBodies(const string&, const PTransformd&,
                    const string&, const PTransformd&,
                    const string&, bool) except +#OutOfRange
    int nrNodes() const
    int nrJoints() const
    void removeJoint(const string&, const string&) except +#OutOfRange
    void removeJointsSV "removeJoints"(const string&, const vector[string]&) except +#OutOfRange
    void mergeSubBodies(const string&, const string&, const map[string,vector[double]]&)
    MultiBody makeMultiBody(const string&, bool, const PTransformd&, const PTransformd&) except +#OutOfRange
    MultiBody makeMultiBody(const string&, JointType, const Vector3d&, const PTransformd&, const PTransformd&) except +#OutOfRange
    map[string,PTransformd] bodiesBaseTransform(const string&, const PTransformd&) except +#OutOfRange
    map[string, vector[string]] successorJoints(const string&) except +#OutOfRange

cdef extern from "<RBDyn/MultiBodyConfig.h>" namespace "rbd":
  cdef cppclass MultiBodyConfig:
    MultiBodyConfig()
    MultiBodyConfig(const MultiBodyConfig &)
    MultiBodyConfig(const MultiBody &)

    void zero(const MultiBody &)

    vector[vector[double]] q
    vector[vector[double]] alpha
    vector[vector[double]] alphaD

    vector[ForceVecd] force

    vector[PTransformd] jointConfig
    vector[MotionVecd] jointVelocity
    vector[vector[double]] jointTorque

    vector[Matrix[double,six,dynamic]] motionSubspace

    vector[PTransformd] bodyPosW
    vector[PTransformd] parentToSon

    vector[MotionVecd] bodyVelW
    vector[MotionVecd] bodyVelB
    vector[MotionVecd] bodyAccB

    Vector3d gravity

  cdef cppclass ConfigConverter:
    ConfigConverter(const ConfigConverter&)

    void sConvert(const MultiBodyConfig&, MultiBodyConfig&) except +

    vector[vector[double]] convertJointVV "convertJoint"(const vector[vector[double]]&)
    vector[double] convertJointV "convertJoint"(const vector[double]&)

  void sParamToVector(const vector[vector[double]]&, VectorXd&) except +
  VectorXd rSParamToVector "sParamToVector"(const MultiBody&, const vector[vector[double]]&) except +
  void sVectorToParam(const VectorXd&, vector[vector[double]] &) except +
  vector[vector[double]] rSVectorToParam "sVectorToParam"(const MultiBody&, const VectorXd&) except +
  VectorXd sDofToVector(const MultiBody&, const vector[vector[double]]&) except +
  vector[vector[double]] sVectorToDof(const MultiBody&, const VectorXd&) except +

cdef extern from "<RBDyn/Jacobian.h>" namespace "rbd":
  cdef cppclass Jacobian:
    Jacobian()
    Jacobian(const Jacobian &)
    Jacobian(const MultiBody&, const string&, const Vector3d&) except +

    MatrixXd sJacobian(const MultiBody&, const MultiBodyConfig&, const PTransformd&) except +
    MatrixXd sJacobian(const MultiBody&, const MultiBodyConfig&) except +
    MatrixXd sJacobianDot(const MultiBody&, const MultiBodyConfig&) except +
    MatrixXd sBodyJacobian(const MultiBody&, const MultiBodyConfig&) except +
    MatrixXd sBodyJacobianDot(const MultiBody&, const MultiBodyConfig&) except +
    MatrixXd sVectorJacobian(const MultiBody&, const MultiBodyConfig&, const Vector3d&) except +
    MatrixXd sVectorBodyJacobian(const MultiBody&, const MultiBodyConfig&, const Vector3d&) except +
    void sTranslateJacobian(const MatrixXd&, const MultiBodyConfig&, const Vector3d&, MatrixXd&) except +
    void sFullJacobian(const MultiBody&, const MatrixXd&, MatrixXd&) except +
    MultiBody sSubMultiBody(const MultiBody&) except +
    vector[int] jointsPath() const
    int dof() const
    Vector3d point() const
    void point(const Vector3d&)
    MotionVecd sVelocity(const MultiBody&, const MultiBodyConfig&) except +
    MotionVecd sVelocity(const MultiBody&, const MultiBodyConfig&, const PTransformd&) except +
    MotionVecd sBodyVelocity(const MultiBody&, const MultiBodyConfig&) except +
    MotionVecd sNormalAcceleration(const MultiBody&, const MultiBodyConfig&) except +
    MotionVecd sNormalAcceleration(const MultiBody&, const MultiBodyConfig&, const PTransformd&, const MotionVecd&) except +
    MotionVecd sNormalAcceleration(const MultiBody&, const MultiBodyConfig&, const vector[MotionVecd]&, const PTransformd&, const MotionVecd&) except +
    MotionVecd sNormalAcceleration(const MultiBody&, const MultiBodyConfig&, const vector[MotionVecd]&) except +
    MotionVecd sBodyNormalAcceleration(const MultiBody&, const MultiBodyConfig&) except +
    MotionVecd sBodyNormalAcceleration(const MultiBody&, const MultiBodyConfig&, const vector[MotionVecd]&) except +

cdef extern from "<RBDyn/FK.h>" namespace "rbd":
  void sForwardKinematics(const MultiBody&, MultiBodyConfig&) except +

cdef extern from "<RBDyn/FV.h>" namespace "rbd":
  void sForwardVelocity(const MultiBody&, MultiBodyConfig&) except +

cdef extern from "<RBDyn/FA.h>" namespace "rbd":
  void sForwardAcceleration(const MultiBody&, MultiBodyConfig&, const MotionVecd&) except +

cdef extern from "<RBDyn/EulerIntegration.h>" namespace "rbd":
  void sEulerIntegration(const MultiBody&, MultiBodyConfig&, double) except +

cdef extern from "<RBDyn/ID.h>" namespace "rbd":
  cdef cppclass InverseDynamics:
    InverseDynamics()
    InverseDynamics(const InverseDynamics &)
    InverseDynamics(const MultiBody &)

    void sInverseDynamics(const MultiBody&, MultiBodyConfig&) except +
    void sInverseDynamicsNoInertia(const MultiBody&, MultiBodyConfig&) except +
    vector[ForceVecd] f() const

cdef extern from "<RBDyn/FD.h>" namespace "rbd":
  cdef cppclass ForwardDynamics:
    ForwardDynamics()
    ForwardDynamics(const ForwardDynamics&)
    ForwardDynamics(const MultiBody&)

    void sForwardDynamics(const MultiBody&, MultiBodyConfig&) except +
    void sComputeH(const MultiBody&, const MultiBodyConfig&) except +
    void sComputeC(const MultiBody&, const MultiBodyConfig&) except +

    MatrixXd H() const
    VectorXd C() const
    vector[RBInertiad] inertiaSubTree() const

cdef extern from "<RBDyn/Coriolis.h>" namespace "rbd":
  cdef cppclass Coriolis:
    Coriolis(const MultiBody&)

    MatrixXd coriolis(const MultiBody&, const MultiBodyConfig&)

cdef extern from "<RBDyn/CoM.h>" namespace "rbd":
  Vector3d sComputeCoM(const MultiBody&, MultiBodyConfig&) except +
  Vector3d sComputeCoMVelocity(const MultiBody&, MultiBodyConfig&) except +
  Vector3d sComputeCoMAcceleration(const MultiBody&, MultiBodyConfig&) except +

  cdef cppclass CoMJacobianDummy:
    CoMJacobianDummy()
    CoMJacobianDummy(const CoMJacobianDummy&)
    CoMJacobianDummy(const MultiBody&)
    CoMJacobianDummy(const MultiBody&, vector[double]) except +

    MatrixXd sJacobian(const MultiBody&, MultiBodyConfig&) except +
    MatrixXd sJacobianDot(const MultiBody&, MultiBodyConfig&) except +

  cdef cppclass CoMJacobian:
    CoMJacobian()
    CoMJacobian(const CoMJacobian&)
    CoMJacobian(const MultiBody&)
    CoMJacobian(const MultiBody&, vector[double]) except +

    MatrixXd sJacobian(const MultiBody&, MultiBodyConfig&) except +
    MatrixXd sJacobianDot(const MultiBody&, MultiBodyConfig&) except +

    void sUpdateInertialParameters(const MultiBody&) except +
    vector[double] weight() const
    void sWeight(const MultiBody&, vector[double]) except +

    Vector3d sVelocity(const MultiBody&, const MultiBodyConfig&) except +
    Vector3d sNormalAcceleration(const MultiBody&, const MultiBodyConfig&) except +
    Vector3d sNormalAcceleration(const MultiBody&, const MultiBodyConfig&, const vector[MotionVecd]&) except +

cdef extern from "<RBDyn/Momentum.h>" namespace "rbd":
  ForceVecd sComputeCentroidalMomentum(const MultiBody&, const MultiBodyConfig&, const Vector3d&) except +
  ForceVecd sComputeCentroidalMomentumDot(const MultiBody&, const MultiBodyConfig&, const Vector3d&, const Vector3d&) except +

  cdef cppclass CentroidalMomentumMatrix:
    CentroidalMomentumMatrix()
    CentroidalMomentumMatrix(const CentroidalMomentumMatrix&)
    CentroidalMomentumMatrix(const MultiBody&)
    CentroidalMomentumMatrix(const MultiBody&, vector[double]) except +

    void sComputeMatrix(const MultiBody&, const MultiBodyConfig&, const Vector3d&) except +
    void sComputeMatrixDot(const MultiBody&, const MultiBodyConfig&, const Vector3d&, const Vector3d&) except +
    void sComputeMatrixAndMatrixDot(const MultiBody&, const MultiBodyConfig&, const Vector3d&, const Vector3d&) except +

    MatrixXd matrix() const
    MatrixXd matrixDot() const

    ForceVecd sMomentum(const MultiBody&, const MultiBodyConfig&, const Vector3d&) except +
    ForceVecd sNormalMomentumDot(const MultiBody&, const MultiBodyConfig&, const Vector3d&, const Vector3d&) except +
    ForceVecd sNormalMomentumDot(const MultiBody&, const MultiBodyConfig&, const Vector3d&, const Vector3d&, const vector[MotionVecd]&) except +

cdef extern from "<RBDyn/ZMP.h>" namespace "rbd":
  Vector3d computeCentroidalZMP(MultiBodyConfig&, Vector3d&, Vector3d&, double)

cdef extern from "<RBDyn/IDIM.h>" namespace "rbd":
  MatrixXd IMPhi(const MotionVecd&)
  VectorXd inertiaToVector(const RBInertiad&)
  RBInertiad sVectorToInertia(const VectorXd&) except +
  VectorXd multiBodyToInertialVector(const MultiBody&)

  cdef cppclass IDIM:
    IDIM()
    IDIM(const IDIM&)
    IDIM(const MultiBody&)

    void sComputeY(const MultiBody&, MultiBodyConfig&) except +
    MatrixXd Y() const
