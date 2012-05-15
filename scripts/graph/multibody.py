import math
import numpy as np

from eigen3 import toNumpy, toEigen, Quaterniond, Vector3d
import spacevecalg as sva
import rbdyn as rbd

from mayavi import mlab
from tvtk.api import tvtk



def setTransform(actor, transform):
  R = transform.rotation()
  T = transform.translation()
  actor.user_transform.set_matrix((R[0], R[1], R[2], T[0],
                                   R[3], R[4], R[5], T[1],
                                   R[6], R[7], R[8], T[2],
                                     0.,   0.,   0.,   1.))

  #actor.user_transform.set_matrix((R[0], R[3], R[6], T[0],
  #                                 R[1], R[4], R[7], T[1],
  #                                 R[2], R[5], R[8], T[2],
  #                                   0.,   0.,   0.,   1.))




class GraphicMultiBody:
  def __init__(self, mb):
    self.bodyS = []
    self.bodyA = []
    self.bodyT = []

    self.comS = []
    self.comA = []

    self.jointS = []
    self.jointA = []
    self.jointT = []

    self.velS = []
    self.velA = []

    self.viewer = mlab.gcf()

    self.transformVel = [(0., 0., 0., 0.)]*mb.nrJoints()

    for i in range(mb.nrBodies()):
      # create links
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
      ca.visibility = 0
      self.viewer.scene.add_actors(ca)

      self.bodyS.append(cs)
      self.bodyA.append(ca)
      self.bodyT.append(sva.PTransform(quat.matrix()))



      # create bodies
      comPos = toNumpy(mb.body(i).inertia().momentum())/mb.body(i).inertia().mass()
      lBody = np.minimum(np.zeros(3).T, comPos)
      uBody = np.maximum(np.zeros(3).T, comPos)

      for j in range(mb.nrJoints()):
        if mb.parent(j) == i:
          t = toNumpy(mb.transform(j).translation())
          lBody = np.minimum(lBody, t)
          uBody = np.maximum(uBody, t)

      sBody = uBody + lBody
      oBody = uBody - lBody

      oBody[oBody < 0.1] = 0.1*mb.body(i).inertia().mass()

      coms = tvtk.CubeSource(x_length=oBody[0,0], y_length=oBody[1,0], z_length=oBody[2,0])
      coms.center = (sBody[0,0]/2., sBody[1,0]/2., sBody[2,0]/2.)


      comm = tvtk.PolyDataMapper(input=coms.output)
      coma = tvtk.Actor(mapper=comm)
      coma.property.color = (0.501, 0., 0.501)
      coma.property.opacity = 0.2
      coma.user_transform = tvtk.Transform()
      coma.visibility = 0
      self.viewer.scene.add_actors(coma)

      self.comS.append(coms)
      self.comA.append(coma)



      # create joints
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
      sa.visibility = 0
      self.viewer.scene.add_actors(sa)

      self.jointS.append(ss)
      self.jointA.append(sa)
      self.jointT.append(st)



      # create velocity arrows
      arrs = tvtk.ArrowSource()
      arrm = tvtk.PolyDataMapper(input=arrs.output)
      arra = tvtk.Actor(mapper=arrm)
      arra.visibility = 0
      self.IMat = arra.matrix
      self.viewer.scene.add_actors(arra)

      self.velS.append(arrs)
      self.velA.append(arra)



  def render(self):
    self.viewer.scene.render()



  def draw(self, mb, mbc):
    bG = list(mbc.bodyPosW)

    for i in range(mb.nrBodies()):
      Xi = bG[i]

      com = toNumpy(mb.body(i).inertia().momentum())/mb.body(i).inertia().mass()
      com = toEigen(com)
      com = Vector3d.Zero()

      comPos = sva.PTransform(com)*Xi

      comA = self.comA[i]
      setTransform(comA, comPos)
      comA.visibility = 1

      if mb.parent(i) != -1:
        Xp = bG[mb.parent(i)]

        # show link from body base to joint
        la = self.bodyA[i]
        setTransform(la, self.bodyT[i]*Xp)
        la.visibility = 1

        # show joint
        sa = self.jointA[i]
        setTransform(sa, self.jointT[i]*Xi)
        sa.visibility = 1




  def drawVel(self, mb, mbc):
    bG = list(mbc.bodyPosW)
    bV = list(mbc.bodyVelW)

    for i in range(mb.nrBodies()):
      a = self.velA[i]

      XiT = bG[i].translation()
      ViL = bV[i].linear()

      arrAxe = np.mat([1., 0., 0.]).T
      nViL = toNumpy(ViL)
      speed = np.linalg.norm(ViL)

      if speed == 0.:
        a.visibility = 0
        continue

      nViL = nViL/speed

      rotAxe = np.cross(arrAxe.T, nViL.T)
      if rotAxe.sum() == 0.:
        rotAxe = np.mat([0., 1., 0.])

      angle = np.rad2deg(math.acos(arrAxe.T*nViL))

      a.position = (XiT[0], XiT[1], XiT[2])
      a.scale = [speed]*3

      a.rotate_wxyz(*self.transformVel[i])
      a.rotate_wxyz(angle, rotAxe[0,0], rotAxe[0,1], rotAxe[0,2])

      self.transformVel[i] = (-angle, rotAxe[0,0], rotAxe[0,1], rotAxe[0,2])

      a.visibility = 1


