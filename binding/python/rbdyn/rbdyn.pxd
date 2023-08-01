#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport eigen.eigen as eigen
cimport sva.sva as sva
cimport rbdyn.c_rbdyn as c_rbdyn
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class DoubleVectorWrapper(object):
    cdef cppbool __own_impl
    cdef object __owner
    cdef vector[double] * v
cdef DoubleVectorWrapper DoubleVectorWrapperFromC(vector[double] &, owner)

cdef class DoubleVectorVectorWrapper(object):
    cdef cppbool __own_impl
    cdef object __owner
    cdef vector[vector[double]] * v

cdef DoubleVectorVectorWrapper DoubleVectorVectorWrapperFromC(
    vector[vector[double]] &, owner
)

cdef class Body(object):
    cdef c_rbdyn.Body impl

cdef Body BodyFromC(const c_rbdyn.Body&)

cdef class BodyVector(object):
    cdef vector[c_rbdyn.Body] v

cdef class Joint(object):
    cdef c_rbdyn.Joint impl

cdef Joint JointFromC(const c_rbdyn.Joint&)

cdef class JointVector(object):
    cdef vector[c_rbdyn.Joint] v

cdef class MultiBody(object):
    cdef c_rbdyn.MultiBody * impl
    cdef cppbool __own_impl

cdef MultiBody MultiBodyFromC(const c_rbdyn.MultiBody&, cppbool copy=?)

cdef class MultiBodyVector(object):
    cdef vector[c_rbdyn.MultiBody] * v
    cdef cppbool __own_impl

cdef MultiBodyVector MultiBodyVectorFromPtr(vector[c_rbdyn.MultiBody]*)

cdef class MultiBodyGraph(object):
    cdef c_rbdyn.MultiBodyGraph * impl
    cdef cppbool __own_impl

cdef MultiBodyGraph MultiBodyGraphFromC(const c_rbdyn.MultiBodyGraph &, cppbool copy=?)

cdef class MultiBodyConfig(object):
    cdef c_rbdyn.MultiBodyConfig * impl
    cdef cppbool __own_impl

cdef MultiBodyConfig MultiBodyConfigFromC(
    const c_rbdyn.MultiBodyConfig&, cppbool copy=?
)

cdef class MultiBodyConfigVector(object):
    cdef vector[c_rbdyn.MultiBodyConfig] * v
    cdef cppbool __own_impl

cdef MultiBodyConfigVector MultiBodyConfigVectorFromPtr(
    vector[c_rbdyn.MultiBodyConfig]*
)

cdef class Jacobian(object):
    cdef c_rbdyn.Jacobian impl

cdef class InverseDynamics(object):
    cdef c_rbdyn.InverseDynamics impl

cdef class ForwardDynamics(object):
    cdef c_rbdyn.ForwardDynamics impl

cdef class Coriolis(object):
    cdef c_rbdyn.Coriolis * impl

cdef class CoMJacobianDummy(object):
    cdef c_rbdyn.CoMJacobianDummy impl

cdef class CoMJacobian(object):
    cdef c_rbdyn.CoMJacobian impl

cdef class CentroidalMomentumMatrix(object):
    cdef c_rbdyn.CentroidalMomentumMatrix impl

cdef class ConfigConverter(object):
    cdef c_rbdyn.ConfigConverter * impl

cdef class IDIM(object):
    cdef c_rbdyn.IDIM impl
