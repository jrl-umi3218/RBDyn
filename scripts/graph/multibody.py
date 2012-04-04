import numpy as np

from tvtk.tools import ivtk
from tvtk.api import tvtk

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

      Xpj = mb.transformFrom(i)*Xp
      Xij = mb.transformTo(i).inv()*Xi

      XpjT = Xpj.translation()
      XijT = Xij.translation()

      # show link from parent body to joint
      ls = tvtk.LineSource(point1=(XpT[0], XpT[1], XpT[2]),
                           point2=(XpjT[0], XpjT[1], XpjT[2]))
      lm = tvtk.PolyDataMapper(input=ls.output)
      la = tvtk.Actor(mapper=lm)
      v.scene.add_actors(la)

      # show link from joint to body
      ls = tvtk.LineSource(point1=(XijT[0], XijT[1], XijT[2]),
                           point2=(XiT[0], XiT[1], XiT[2]))
      lm = tvtk.PolyDataMapper(input=ls.output)
      la = tvtk.Actor(mapper=lm)
      v.scene.add_actors(la)

      # show joint
      ss = tvtk.SphereSource(center=(XijT[0], XijT[1], XijT[2]),
                             radius=0.05)
      sm = tvtk.PolyDataMapper(input=ss.output)
      sa = tvtk.Actor(mapper=sm)
      v.scene.add_actors(sa)


    #ss = tvtk.SphereSource(center=(XiT[0], XiT[1], XiT[2]),
    #                       radius=0.1)
    #sm = tvtk.PolyDataMapper(input=ss.output)
    #sa = tvtk.Actor(mapper=sm)

    #v.scene.add_actors(sa)
    a = tvtk.Axes()
    a.scale_factor = 0.1
    #a.origin = np.array([XiT[0], XiT[1], XiT[2]])
    am = tvtk.PolyDataMapper(input=a.output)
    aa = tvtk.Actor(mapper=am)
    v.scene.add_actors(aa)

  return v

