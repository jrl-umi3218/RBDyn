#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from rbdyn.parsers.c_parsers cimport *

cdef extern from "rbdyn_parsers_wrapper.hpp" namespace "rbdyn_parsers":
    const GeometryMesh& getMesh(const Geometry&)
    const GeometryBox& getBox(const Geometry&)
    const GeometryCylinder& getCylinder(const Geometry&)
    const GeometrySphere& getSphere(const Geometry&)
    const GeometrySuperellipsoid& getSuperellipsoid(const Geometry&)
    void setMesh(Geometry&, const GeometryMesh&)
    void setBox(Geometry&, const GeometryBox&)
    void setCylinder(Geometry&, const GeometryCylinder&)
    void setSphere(Geometry&, const GeometrySphere&)
    void setSuperellipsoid(Geometry&, const GeometrySuperellipsoid&)
