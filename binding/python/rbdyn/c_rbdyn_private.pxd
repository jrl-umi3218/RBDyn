#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from rbdyn.c_rbdyn cimport *
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool

cdef extern from "rbdyn_wrapper.hpp" namespace "rbd":
    string BodyToString(const Body &)
    string JointToString(const Joint &)
    vector[double] ZeroParam(JointType)
    vector[double] ZeroDof(JointType)
    ConfigConverter* ConfigConverterConstructor(
        const MultiBody&,
        const MultiBody&
    ) except +
    MultiBody& const_cast_mb(const MultiBody&)
    MultiBodyConfig& const_cast_mbc(const MultiBodyConfig&)
    MultiBodyGraph& const_cast_mbg(const MultiBodyGraph&)
    void dv_set_item(vector[double] & v, unsigned int idx, double value)
    void dvv_set_item(
        vector[vector[double]] & v, unsigned int idx, const vector[double] & value
    )
    void mbcv_set_item(vector[MultiBodyConfig]&, unsigned int, const MultiBodyConfig&)
