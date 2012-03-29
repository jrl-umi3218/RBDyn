import numpy as np

from tvtk.tools import ivtk
from tvtk.api import tvtk

def show_multibody(mb, mbc):
  bG = [i for i in mbc.bodyGlobal]

  v = ivtk.IVTKWithCrustAndBrowser(size=(600,600))
  v.open()

  for i in range(mb.nrBodies()):
    Xi = bG[i]
    XiT = Xi.translation()

    if mb.parent(i) != -1:
      Xp = bG[mb.parent(i)]
      XpT = Xp.translation()

      ls = tvtk.LineSource(point1=(XpT[0], XpT[1], XpT[2]),
                           point2=(XiT[0], XiT[1], XiT[2]))
      lm = tvtk.PolyDataMapper(input=ls.output)
      la = tvtk.Actor(mapper=lm)
      v.scene.add_actors(la)

    ss = tvtk.SphereSource(center=(XiT[0], XiT[1], XiT[2]),
                           radius=0.1)
    sm = tvtk.PolyDataMapper(input=ss.output)
    sa = tvtk.Actor(mapper=sm)

    v.scene.add_actors(sa)

