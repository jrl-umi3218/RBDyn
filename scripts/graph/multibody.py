import math
import numpy as np

from eigen3 import toNumpy

from mayavi import mlab
from tvtk.api import tvtk

class GraphicMultiBody:
  def __init__(self, mb):
    self.bodyS = []
    self.bodyA = []

    self.jointS = []
    self.jointA = []

    self.velS = []
    self.velA = []

    self.viewer = mlab.gcf()

    self.transformVel = [(0., 0., 0., 0.)]*mb.nrJoints()

    for i in range(mb.nrBodies()):
      ls = tvtk.LineSource()
      lm = tvtk.PolyDataMapper(input=ls.output)
      la = tvtk.Actor(mapper=lm)
      la.visibility = 0
      self.viewer.scene.add_actors(la)

      self.bodyS.append(ls)
      self.bodyA.append(la)

      ss = tvtk.SphereSource(radius=0.05)
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

      if mb.parent(i) != -1:
        Xp = bG[mb.parent(i)]
        XpT = Xp.translation()

        # show link from body base to joint
        ls = self.bodyS[i]
        ls.point1 = (XpT[0], XpT[1], XpT[2])
        ls.point2 = (XiT[0], XiT[1], XiT[2])

        la = self.bodyA[i]
        la.visibility = 1

        # show joint
        ss = self.jointS[i]
        ss.center=(XiT[0], XiT[1], XiT[2])

        sa = self.jointA[i]
        sa.visibility = 1




  def drawVel(self, mb, mbc):
    bG = list(mbc.bodyPosW)
    bV = list(mbc.bodyVelW)

    for i in range(mb.nrBodies()):
      XiT = bG[i].translation()
      ViL = bV[i].linear()

      arrAxe = np.mat([1., 0., 0.]).T
      nViL = toNumpy(ViL)

      rotAxe = np.cross(arrAxe.T, nViL.T)
      if rotAxe.sum() == 0.:
        rotAxe = np.mat([0., 1., 0.])
      angle = np.rad2deg(math.acos(arrAxe.T*nViL))

      speed = np.linalg.norm(toNumpy(ViL))

      a = self.velA[i]

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

