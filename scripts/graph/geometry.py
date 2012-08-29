import numpy as np

from eigen3 import toNumpy, Quaterniond, Vector3d

import spacevecalg as sva
import rbdyn as rbd

from tvtk.api import tvtk

from wavefront_obj import readObj

class Geometry(object):
  def joint(self, pos):
    return None

  def body(self, pos):
    return None

  def link(self, pos):
    return None



class DefaultGeometry(Geometry):
  def __init__(self, mb):
    self._createLinks(mb)
    self._createBodies(mb)
    self._createJoints(mb)



  def _createLinks(self, mb):
    self.links = []

    for i in range(mb.nrBodies()):
      cs = tvtk.CylinderSource()

      cs.height = np.linalg.norm(toNumpy(mb.transform(i).translation()))
      cs.radius = 0.01
      cs.center = (0., cs.height/2., 0.)

      quat = Quaterniond()
      if np.linalg.norm(toNumpy(mb.transform(i).translation())) == 0.:
        quat.setIdentity()
      else:
        quat.setFromTwoVectors(mb.transform(i).translation(), Vector3d.UnitY())

      cm = tvtk.PolyDataMapper(input=cs.output)

      ca = tvtk.Actor(mapper=cm)
      ca.property.color = (0., 1., 1.)
      ca.user_transform = tvtk.Transform()

      self.links.append((ca, sva.PTransform(quat.matrix())))



  def _createBodies(self, mb):
    self.bodies = []

    for i in range(mb.nrBodies()):
      bodyPos = toNumpy(mb.body(i).inertia().momentum())/mb.body(i).inertia().mass()
      lBody = np.minimum(np.zeros(3).T, bodyPos)
      uBody = np.maximum(np.zeros(3).T, bodyPos)

      for j in range(mb.nrJoints()):
        if mb.parent(j) == i:
          t = toNumpy(mb.transform(j).translation())
          lBody = np.minimum(lBody, t)
          uBody = np.maximum(uBody, t)

      sBody = uBody + lBody
      oBody = uBody - lBody

      oBody[oBody < 0.1] = 0.1*mb.body(i).inertia().mass()

      bodys = tvtk.CubeSource(x_length=oBody[0,0], y_length=oBody[1,0], z_length=oBody[2,0])
      bodys.center = (sBody[0,0]/2., sBody[1,0]/2., sBody[2,0]/2.)

      bodym = tvtk.PolyDataMapper(input=bodys.output)

      bodya = tvtk.Actor(mapper=bodym)
      bodya.property.color = (0.501, 0., 0.501)
      bodya.property.opacity = 0.2
      bodya.user_transform = tvtk.Transform()

      self.bodies.append((bodya, sva.PTransform.Identity()))



  def _createJoints(self, mb):
    self.joints = []

    for i in range(mb.nrBodies()):

      def jointGeom(type):
        if type == rbd.Joint.RevX:
          s = tvtk.CylinderSource(height=0.1, radius=0.02)
          t = sva.PTransform(sva.RotZ(np.pi/2.))
          color = (1., 0., 0.)
        elif type == rbd.Joint.RevY:
          s = tvtk.CylinderSource(height=0.1, radius=0.02)
          t = sva.PTransform.Identity()
          color = (0., 1., 0.)
        elif type == rbd.Joint.RevZ:
          s = tvtk.CylinderSource(height=0.1, radius=0.02)
          t = sva.PTransform(sva.RotX(np.pi/2.))
          color = (0., 0., 1.)
        else:
          s = tvtk.SphereSource(radius=0.02)
          t = sva.PTransform.Identity()
          color = (1., 1., 1.)
        return s, t, color

      ss, st, color = jointGeom(mb.joint(i).type())

      sm = tvtk.PolyDataMapper(input=ss.output)
      sa = tvtk.Actor(mapper=sm)
      sa.property.color = color
      sa.user_transform = tvtk.Transform()

      self.joints.append((sa, st))



  def joint(self, pos):
    return self.joints[pos]


  def body(self, pos):
    return self.bodies[pos]


  def link(self, pos):
    return self.links[pos]



class MeshGeometry(DefaultGeometry):
  def __init__(self, mb, files, opacity=1.):
    self._createLinks(mb)
    self._createBodies(mb, files, opacity)
    self._createJoints(mb)


  def _createBodies(self, mb, files, opacity):
    self.bodies = []

    for i in range(mb.nrBodies()):
      vert = np.empty((0,3))
      face = np.empty((0,3))
      color = []
      for objfile in files[mb.body(i).id()]:
        v, f, c = readObj(objfile)
        face = np.vstack((face, f + vert.shape[0]))
        vert = np.vstack((vert, v))
        color.extend(c)

      vtkColor = tvtk.UnsignedCharArray()
      vtkColor.from_array(color)

      bodys = tvtk.PolyData(points=vert, polys=face)
      bodys.cell_data.scalars = vtkColor
      bodym = tvtk.PolyDataMapper(input=bodys)

      bodya = tvtk.Actor(mapper=bodym)
      bodya.property.opacity = opacity
      bodya.user_transform = tvtk.Transform()

      self.bodies.append((bodya, sva.PTransform.Identity()))


