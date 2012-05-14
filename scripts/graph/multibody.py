import math
import numpy as np

from eigen3 import toNumpy, toEigen, Quaterniond, Vector3d
import spacevecalg as sva

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

    self.velS = []
    self.velA = []

    self.viewer = mlab.gcf()

    self.transformVel = [(0., 0., 0., 0.)]*mb.nrJoints()

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
      ca.visibility = 0
      self.viewer.scene.add_actors(ca)

      self.bodyS.append(cs)
      self.bodyA.append(ca)
      self.bodyT.append(sva.PTransform(quat.matrix()))



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



      ss = tvtk.SphereSource(radius=0.02)
      sm = tvtk.PolyDataMapper(input=ss.output)
      sa = tvtk.Actor(mapper=sm)
      sa.visibility = 0
      self.viewer.scene.add_actors(sa)

      self.jointS.append(ss)
      self.jointA.append(sa)



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
      XiT = Xi.translation()

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
        sa.position = (XiT[0], XiT[1], XiT[2])
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






def show_multibody(mb, mbc):
  bG = [i for i in mbc.bodyPosW]

  v = mlab.gcf()

  for i in range(mb.nrBodies()):
    Xi = bG[i]
    XiT = Xi.translation()

    if mb.parent(i) != -1:
      Xp = bG[mb.parent(i)]
      XpT = Xp.translation()

      # show link from body base to joint
      ls = tvtk.LineSource(point1=(XpT[0], XpT[1], XpT[2]),
                           point2=(XiT[0], XiT[1], XiT[2]))
      lm = tvtk.PolyDataMapper(input=ls.output)
      la = tvtk.Actor(mapper=lm)
      v.scene.add_actors(la)

      # show joint
      ss = tvtk.SphereSource(center=(XiT[0], XiT[1], XiT[2]),
                             radius=0.05)
      sm = tvtk.PolyDataMapper(input=ss.output)
      sa = tvtk.Actor(mapper=sm)
      v.scene.add_actors(sa)


    #v.scene.add_actors(sa)
    a = tvtk.Axes()
    a.scale_factor = 0.1
    #a.origin = np.array([XiT[0], XiT[1], XiT[2]])
    am = tvtk.PolyDataMapper(input=a.output)
    aa = tvtk.Actor(mapper=am)
    v.scene.add_actors(aa)

  return v

