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

cdef extern from "<RBDyn/parsers/common.h>" namespace "rbd":
  cdef cppclass Limits:
    cppmap[string, vector[double]] lower
    cppmap[string, vector[double]] upper
    cppmap[string, vector[double]] velocity
    cppmap[string, vector[double]] torque

  cdef cppclass GeometryMesh "rbd::Geometry::Mesh":
    string filename
    double scale

  cdef cppclass GeometryBox "rbd::Geometry::Box":
    Vector3d size

  cdef cppclass GeometryCylinder "rbd::Geometry::Cylinder":
    double radius
    double length

  cdef cppclass GeometrySphere "rbd::Geometry::Sphere":
    double radius

  cdef cppclass GeometrySuperellipsoid "rbd::Geometry::Superellipsoid":
    Vector3d size
    double epsilon1
    double epsilon2

  ctypedef enum GeometryType "rbd::Geometry::Type":
    GeometryBOX "rbd::Geometry::BOX"
    GeometryCYLINDER "rbd::Geometry::CYLINDER"
    GeometrySPHERE "rbd::Geometry::SPHERE"
    GeometrySUPERELLIPSOID "rbd::Geometry::SUPERELLIPSOID"
    GeometryMESH "rbd::Geometry::MESH"
    GeometryUNKNOWN "rbd::Geometry::UNKNOWN"

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
    cppmap[string, PTransformd] collision_tf
    string name

  ParserResult from_file(const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool)

cdef extern from "<RBDyn/parsers/urdf.h>" namespace "rbd":
  ParserResult from_urdf(const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool)
  ParserResult from_urdf_file(const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool)

cdef extern from "<RBDyn/parsers/yaml.h>" namespace "rbd":
  ParserResult from_yaml(const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool)
  ParserResult from_yaml_file(const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool)
