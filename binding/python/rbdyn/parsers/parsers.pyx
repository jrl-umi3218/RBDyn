# distutils: language = c++

#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport rbdyn.parsers.c_parsers as c_parsers
cimport rbdyn.parsers.c_parsers_private as c_parsers_private

cimport eigen.eigen as eigen
cimport sva.sva as sva
cimport rbdyn.rbdyn as rbdyn


cdef class Limits(object):
    def __cinit__(self, *args):
        pass
    property lower:
        def __get__(self):
            return self.impl.lower
    property upper:
        def __get__(self):
            return self.impl.upper
    property velocity:
        def __get__(self):
            return self.impl.velocity
    property torque:
        def __get__(self):
            return self.impl.torque

cdef Limits LimitsFromC(const c_parsers.Limits & lim):
    cdef Limits ret = Limits()
    ret.impl = lim
    return ret

cdef class GeometryMesh(object):
    def __cinit__(self, *args):
        pass

    property filename:
        def __get__(self):
            return self.impl.filename
    property scaleV:
        def __get__(self):
            return eigen.Vector3dFromC(self.impl.scaleV)

    def __richcmp__(GeometryMesh self, GeometryMesh other, int op):
        if op == 2:
            return (
                self.impl.filename == other.impl.filename
                and self.impl.scaleV == other.impl.scaleV
            )
        elif op == 3:
            return (
                self.impl.filename != other.impl.filename
                and self.impl.scaleV != other.impl.scaleV
            )
        else:
            raise NotImplementedError("This comparison is not supported")

cdef GeometryMesh GeometryMeshFromC(const c_parsers.GeometryMesh & m):
    cdef GeometryMesh ret = GeometryMesh()
    ret.impl = m
    return ret

cdef class GeometryBox(object):
    def __cinit__(self, *args):
        pass

    property size:
        def __get__(self):
            return eigen.Vector3dFromC(self.impl.size)

    def __richcmp__(GeometryBox self, GeometryBox other, int op):
        if op == 2:
            return self.impl.size == other.impl.size
        elif op == 3:
            return self.impl.size != other.impl.size
        else:
            raise NotImplementedError("This comparison is not supported")

cdef GeometryBox GeometryBoxFromC(const c_parsers.GeometryBox & m):
    cdef GeometryBox ret = GeometryBox()
    ret.impl = m
    return ret

cdef class GeometryCylinder(object):
    def __cinit__(self, *args):
        pass

    property radius:
        def __get__(self):
            return self.impl.radius
    property length:
        def __get__(self):
            return self.impl.length

    def __richcmp__(GeometryCylinder self, GeometryCylinder other, int op):
        if op == 2:
            return (
                self.impl.radius == other.radius
                and self.impl.length == other.impl.length
            )
        elif op == 3:
            return (
                self.impl.radius != other.radius
                and self.impl.length != other.impl.length
            )
        else:
            raise NotImplementedError("This comparison is not supported")

cdef GeometryCylinder GeometryCylinderFromC(const c_parsers.GeometryCylinder & m):
    cdef GeometryCylinder ret = GeometryCylinder()
    ret.impl = m
    return ret

cdef class GeometrySphere(object):
    def __cinit__(self, *args):
        pass

    property radius:
        def __get__(self):
            return self.impl.radius

    def __richcmp__(GeometrySphere self, GeometrySphere other, int op):
        if op == 2:
            return self.impl.radius == other.impl.radius
        elif op == 3:
            return self.impl.radius != other.impl.radius
        else:
            raise NotImplementedError("This comparison is not supported")

cdef GeometrySphere GeometrySphereFromC(const c_parsers.GeometrySphere & m):
    cdef GeometrySphere ret = GeometrySphere()
    ret.impl = m
    return ret

cdef class GeometrySuperellipsoid(object):
    def __cinit__(self, *args):
        pass

    property size:
        def __get__(self):
            return eigen.Vector3dFromC(self.impl.size)
    property epsilon1:
        def __get__(self):
            return self.impl.epsilon1
    property epsilon2:
        def __get__(self):
            return self.impl.epsilon2

    def __richcmp__(GeometrySuperellipsoid self, GeometrySuperellipsoid other, int op):
        if op == 2:
            return (
                self.impl.size == other.impl.size
                and self.impl.epsilon1 == other.impl.epsilon1
                and self.impl.epsilon2 == other.impl.epsilon2
            )
        elif op == 3:
            return (
                self.impl.size != other.impl.size
                and self.impl.epsilon1 != other.impl.epsilon1
                and self.impl.epsilon2 != other.impl.epsilon2
            )
        else:
            raise NotImplementedError("This comparison is not supported")

cdef GeometrySuperellipsoid GeometrySuperellipsoidFromC(
    const c_parsers.GeometrySuperellipsoid & m
):
    cdef GeometrySuperellipsoid ret = GeometrySuperellipsoid()
    ret.impl = m
    return ret

cdef class Geometry(object):
    MESH = c_parsers.GeometryMESH
    BOX = c_parsers.GeometryBOX
    CYLINDER = c_parsers.GeometryCYLINDER
    SPHERE = c_parsers.GeometrySPHERE
    SUPERELLIPSOID = c_parsers.GeometrySUPERELLIPSOID
    UNKNOWN = c_parsers.GeometryUNKNOWN

    def __cinit__(self, *args):
        pass

    property type:
        def __get__(self):
            return self.impl._type

    def __set_geom_mesh(self, GeometryMesh value):
        assert(self.impl._type == c_parsers.GeometryMESH)
        c_parsers_private.setMesh(self.impl, value.impl)

    def __set_geom_box(self, GeometryBox value):
        assert(self.impl._type == c_parsers.GeometryBOX)
        c_parsers_private.setBox(self.impl, value.impl)

    def __set_geom_cylinder(self, GeometryCylinder value):
        assert(self.impl._type == c_parsers.GeometryCYLINDER)
        c_parsers_private.setCylinder(self.impl, value.impl)

    def __set_geom_sphere(self, GeometrySphere value):
        assert(self.impl._type == c_parsers.GeometrySPHERE)
        c_parsers_private.setSphere(self.impl, value.impl)

    def __set_geom_superellipsoid(self, GeometrySuperellipsoid value):
        assert(self.impl._type == c_parsers.GeometrySUPERELLIPSOID)
        c_parsers_private.setSuperellipsoid(self.impl, value.impl)

    property data:
        def __get__(self):
            if self.impl._type == c_parsers.GeometryMESH:
                return GeometryMeshFromC(c_parsers_private.getMesh(self.impl))
            elif self.impl._type == c_parsers.GeometryBOX:
                return GeometryBoxFromC(c_parsers_private.getBox(self.impl))
            elif self.impl._type == c_parsers.GeometryCYLINDER:
                return GeometryCylinderFromC(c_parsers_private.getCylinder(self.impl))
            elif self.impl._type == c_parsers.GeometrySPHERE:
                return GeometrySphereFromC(c_parsers_private.getSphere(self.impl))
            elif self.impl._type == c_parsers.GeometrySUPERELLIPSOID:
                return GeometrySuperellipsoidFromC(c_parsers_private.getSuperellipsoid(
                    self.impl)
                )
            else:
                print("No data in the Geometry you are accessing")
                return None

    def __richcmp__(Geometry self, Geometry other, int op):
        if op == 2:
            return self.impl._type == other.impl._type and self.data == other.data
        elif op == 3:
            return self.impl._type != other.impl._type and self.data != other.data
        else:
            raise NotImplementedError("This comparison is not supported")

cdef Geometry GeometryFromC(const c_parsers.Geometry & g):
    cdef Geometry ret = Geometry()
    ret.impl = g
    return ret

cdef class Visual(object):
    def __cinit__(self, *args):
        pass
    property name:
        def __get__(self):
            return self.impl.name
    property origin:
        def __get__(self):
            return sva.PTransformdFromC(self.impl.origin)
    property geometry:
        def __get__(self):
            return GeometryFromC(self.impl.geometry)

    def __richcmp__(Visual self, Visual other, int op):
        if op == 2:
            return (
                self.impl.name == other.impl.name
                and self.impl.origin == other.impl.origin
                and self.geometry == other.geometry
            )
        elif op == 3:
            return (
                self.impl.name != other.impl.name
                and self.impl.origin != other.impl.origin
                and self.geometry != other.geometry
            )
        else:
            raise NotImplementedError("This comparison is not supported")

cdef Visual VisualFromC(const c_parsers.Visual& v):
    cdef Visual ret = Visual()
    ret.impl = v
    return ret

cdef class ParserResult(object):
    def __cinit__(self, *args):
        pass
    property mb:
        def __get__(self):
            return rbdyn.MultiBodyFromC(self.impl.mb, copy = False)
    property mbc:
        def __get__(self):
            return rbdyn.MultiBodyConfigFromC(self.impl.mbc, copy = False)
    property mbg:
        def __get__(self):
            return rbdyn.MultiBodyGraphFromC(self.impl.mbg, copy = False)
    property limits:
        def __get__(self):
            return LimitsFromC(self.impl.limits)
    property visual:
        def __get__(self):
            res = {}
            for it in self.impl.visual:
                res[it.first] = []
                for v in it.second:
                    res[it.first].append(VisualFromC(v))
            return res
    property collision:
        def __get__(self):
            res = {}
            for it in self.impl.collision:
                res[it.first] = []
                for v in it.second:
                    res[it.first].append(VisualFromC(v))
            return res
    property name:
        def __get__(self):
            return self.impl.name

cdef ParserResult ParserResultFromC(const c_parsers.ParserResult & u):
    cdef ParserResult res = ParserResult()
    res.impl = u
    return res


def from_urdf(
    content,
    fixed = True,
    filteredLinks = None,
    transformInertia = True,
    baseLink = "",
    withVirtualLinks = True
):
    if isinstance(content, unicode):
        content = content.encode(u"ascii")
    if isinstance(baseLink, unicode):
        baseLink = baseLink.encode(u"ascii")
    if filteredLinks is not None:
        filteredLinks = [
            fL.encode("ascii") if isinstance(fL, unicode) else fL
            for fL in filteredLinks
        ]
    else:
        filteredLinks = []
    return ParserResultFromC(c_parsers.from_urdf(
        content, fixed, filteredLinks, transformInertia, baseLink, withVirtualLinks)
    )


def from_urdf_file(
    content,
    fixed = True,
    filteredLinks = None,
    transformInertia = True,
    baseLink = "",
    withVirtualLinks = True
):
    if isinstance(content, unicode):
        content = content.encode(u"ascii")
    if isinstance(baseLink, unicode):
        baseLink = baseLink.encode(u"ascii")
    if filteredLinks is not None:
        filteredLinks = [
            fL.encode("ascii") if isinstance(fL, unicode) else fL
            for fL in filteredLinks
        ]
    else:
        filteredLinks = []
    return ParserResultFromC(c_parsers.from_urdf_file(
        content, fixed, filteredLinks, transformInertia, baseLink, withVirtualLinks)
    )


def from_yaml(
    content,
    fixed = True,
    filteredLinks = None,
    transformInertia = True,
    baseLink = "",
    withVirtualLinks = True
):
    if isinstance(content, unicode):
        content = content.encode(u"ascii")
    if isinstance(baseLink, unicode):
        baseLink = baseLink.encode(u"ascii")
    if filteredLinks is not None:
        filteredLinks = [
            fL.encode("ascii") if isinstance(fL, unicode) else fL
            for fL in filteredLinks
        ]
    else:
        filteredLinks = []
    return ParserResultFromC(c_parsers.from_yaml(
        content, fixed, filteredLinks, transformInertia, baseLink, withVirtualLinks)
    )


def from_yaml_file(
    content,
    fixed = True,
    filteredLinks = None,
    transformInertia = True,
    baseLink = "",
    withVirtualLinks = True
):
    if isinstance(content, unicode):
        content = content.encode(u"ascii")
    if isinstance(baseLink, unicode):
        baseLink = baseLink.encode(u"ascii")
    if filteredLinks is not None:
        filteredLinks = [
            fL.encode("ascii") if isinstance(fL, unicode) else fL
            for fL in filteredLinks
        ]
    else:
        filteredLinks = []
    return ParserResultFromC(c_parsers.from_yaml_file(
        content, fixed, filteredLinks, transformInertia, baseLink, withVirtualLinks)
    )


def from_file(
    fname,
    fixed = True,
    filteredLinks = None,
    transformInertia = True,
    baseLink = "",
    withVirtualLinks = True
):
    if isinstance(fname, unicode):
        fname = fname.encode(u"ascii")
    if isinstance(baseLink, unicode):
        baseLink = baseLink.encode(u"ascii")
    if filteredLinks is not None:
        filteredLinks = [
            fL.encode("ascii") if isinstance(fL, unicode) else fL
            for fL in filteredLinks
        ]
    else:
        filteredLinks = []
    return ParserResultFromC(c_parsers.from_file(
        fname, fixed, filteredLinks, transformInertia, baseLink, withVirtualLinks)
    )
