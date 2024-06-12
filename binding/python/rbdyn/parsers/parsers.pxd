#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport rbdyn.parsers.c_parsers as c_parsers

cdef class Limits(object):
    cdef c_parsers.Limits impl

cdef Limits LimitsFromC(const c_parsers.Limits&)

cdef class GeometryMesh(object):
    cdef c_parsers.GeometryMesh impl

cdef GeometryMesh GeometryMeshFromC(const c_parsers.GeometryMesh&)

cdef class GeometryBox(object):
    cdef c_parsers.GeometryBox impl

cdef GeometryBox GeometryBoxFromC(const c_parsers.GeometryBox&)

cdef class GeometryCylinder(object):
    cdef c_parsers.GeometryCylinder impl

cdef GeometryCylinder GeometryCylinderFromC(const c_parsers.GeometryCylinder&)

cdef class GeometrySphere(object):
    cdef c_parsers.GeometrySphere impl

cdef GeometrySphere GeometrySphereFromC(const c_parsers.GeometrySphere&)

cdef class GeometrySuperellipsoid(object):
    cdef c_parsers.GeometrySuperellipsoid impl

cdef GeometrySuperellipsoid GeometrySuperellipsoidFromC(
    const c_parsers.GeometrySuperellipsoid&
)

cdef class Geometry(object):
    cdef c_parsers.Geometry impl

cdef Geometry GeometryFromC(const c_parsers.Geometry&)

cdef class Visual(object):
    cdef c_parsers.Visual impl

cdef Visual VisualFromC(const c_parsers.Visual&)

cdef class ParserResult(object):
    cdef c_parsers.ParserResult impl

cdef ParserResult ParserResultFromC(const c_parsers.ParserResult&)
