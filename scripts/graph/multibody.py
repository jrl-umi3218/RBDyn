import numpy as np

from tvtk.tools import ivtk
from tvtk.api import tvtk

class GraphicMultiBody:
  def __init__(self, mb, v=None):
    self.bodyS = []
    self.bodyA = []

    self.jointS = []
    self.jointA = []

    self.viewer = v if v is not None else ivtk.IVTKWithCrustAndBrowser(size=(600,600))
    self.viewer.open()

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

    self.viewer.scene.render()




def show_multibody(mb, mbc, v=None):
  bG = [i for i in mbc.bodyPosW]

  v = v if v is not None else ivtk.IVTKWithCrustAndBrowser(size=(600,600))
  v.open()

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

