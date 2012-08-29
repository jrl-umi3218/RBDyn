import math
import numpy as np

from eigen3 import toNumpy, toEigen, Vector3d
import spacevecalg as sva

from mayavi import mlab
from tvtk.api import tvtk

import geometry



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
  def __init__(self, mb, geom=None, staticTransform=sva.PTransform.Identity()):
    self.viewer = mlab.gcf()

    if geom == None:
      self.geom = geometry.DefaultGeometry(mb)
    else:
      self.geom = geom

    self.staticTransform = staticTransform

    self.transformVel = [(0., 0., 0., 0.)]*mb.nrJoints()

    actors = []
    for i in range(mb.nrBodies()):
      actors.append(self.geom.body(i)[0])
      if mb.parent(i) != -1:
        actors.append(self.geom.link(i)[0])
        actors.append(self.geom.joint(i)[0])

    self.viewer.scene.add_actors(actors)


    # todo : Replace by fixture
    # create velocity arrows
    self.velA = []
    for i in range(mb.nrBodies()):
      arrs = tvtk.ArrowSource()
      arrm = tvtk.PolyDataMapper(input=arrs.output)
      arra = tvtk.Actor(mapper=arrm)
      arra.visibility = 0
      self.IMat = arra.matrix
      self.viewer.scene.add_actors(arra)

      self.velA.append(arra)


  def render(self):
    self.viewer.scene.render()



  def draw(self, mb, mbc):
    bG = list(mbc.bodyPosW)

    for i in range(mb.nrBodies()):
      Xi = bG[i]

      #com = toNumpy(mb.body(i).inertia().momentum())/mb.body(i).inertia().mass()
      #com = toEigen(com)
      com = Vector3d.Zero()

      comPos = sva.PTransform(com)*Xi*self.staticTransform

      comA, comS = self.geom.body(i)
      setTransform(comA, comPos)

      if mb.parent(i) != -1:
        Xp = bG[mb.parent(i)]

        # show link from body base to joint
        linkA, linkS = self.geom.link(i)
        setTransform(linkA, linkS*Xp*self.staticTransform)

        # show joint
        jointA, jointS = self.geom.joint(i)
        setTransform(jointA, jointS*Xi*self.staticTransform)



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


