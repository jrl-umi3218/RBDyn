#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen cimport *
from sva.c_sva cimport PTransformd
from rbdyn.c_rbdyn cimport MultiBody, MultiBodyConfig, MultiBodyGraph

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<RBDyn/parsers/common.h>" namespace "rbd::parsers":
    cdef cppclass Limits:
        cppmap[string, vector[double]] lower
        cppmap[string, vector[double]] upper
        cppmap[string, vector[double]] velocity
        cppmap[string, vector[double]] torque

    cdef cppclass GeometryMesh "rbd::parsers::Geometry::Mesh":
        string filename
        Vector3d scaleV

    cdef cppclass GeometryBox "rbd::parsers::Geometry::Box":
        Vector3d size

    cdef cppclass GeometryCylinder "rbd::parsers::Geometry::Cylinder":
        double radius
        double length

    cdef cppclass GeometrySphere "rbd::parsers::Geometry::Sphere":
        double radius

    cdef cppclass GeometrySuperellipsoid "rbd::parsers::Geometry::Superellipsoid":
        Vector3d size
        double epsilon1
        double epsilon2

    ctypedef enum GeometryType "rbd::parsers::Geometry::Type":
        GeometryBOX "rbd::parsers::Geometry::BOX"
        GeometryCYLINDER "rbd::parsers::Geometry::CYLINDER"
        GeometrySPHERE "rbd::parsers::Geometry::SPHERE"
        GeometrySUPERELLIPSOID "rbd::parsers::Geometry::SUPERELLIPSOID"
        GeometryMESH "rbd::parsers::Geometry::MESH"
        GeometryUNKNOWN "rbd::parsers::Geometry::UNKNOWN"

    cdef cppclass Geometry:
        GeometryType _type "type"
        # data is boost::variant so it is exposed through the wrapper

    cdef cppclass Visual:
        string name
        PTransformd origin
        Geometry geometry

    cdef cppclass ParserResult:
        MultiBody mb
        MultiBodyConfig mbc
        MultiBodyGraph mbg
        Limits limits
        cppmap[string, vector[Visual]] visual
        cppmap[string, vector[Visual]] collision
        string name

    ParserResult from_file(
        const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool
    )

cdef extern from "<RBDyn/parsers/urdf.h>" namespace "rbd::parsers":
    ParserResult from_urdf(
        const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool
    )
    ParserResult from_urdf_file(
        const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool
    )

cdef extern from "<RBDyn/parsers/yaml.h>" namespace "rbd::parsers":
    ParserResult from_yaml(
        const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool
    )
    ParserResult from_yaml_file(
        const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool
    )
