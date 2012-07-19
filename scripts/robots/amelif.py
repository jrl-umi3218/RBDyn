import os.path
from xml.dom.minidom import parse

import numpy as np

from eigen3 import Vector3d, Vector6d, toEigen

import spacevecalg as sva
import rbdyn as rbd


def jointType(type, axis):
  if type == 'revolute':
    if axis == 'x':
      return rbd.Joint.RevX
    elif axis == 'y':
      return rbd.Joint.RevY
    elif axis == 'z':
      return rbd.Joint.RevZ
  elif type == 'prismatic':
    if axis == 'x':
      return rbd.Joint.PrismX
    elif axis == 'y':
      return rbd.Joint.PrismY
    elif axis == 'z':
      return rbd.Joint.PrismZ
  elif type == 'spherical':
    return rbd.Joint.Spherical
  elif type == 'fixed':
    return rbd.Joint.Fixed




def make_amelif_robot(file, isFixed):
  mbg = rbd.MultiBodyGraph()

  xmlpath = os.path.dirname(file)
  sep = os.path.sep
  objpath = xmlpath + sep + '..' + sep + 'mesh' + sep
  dom = parse(file)

  objFiles = {}

  for bodyNode in dom.getElementsByTagName('Body'):
    id = int(bodyNode.getAttribute('id'))

    label = bodyNode.getElementsByTagName('Label')[0].firstChild.data
    mass = float(bodyNode.getElementsByTagName('Mass')[0].firstChild.data)
    comL = bodyNode.getElementsByTagName('CoM')[0].firstChild.data.split()
    com = Vector3d(float(comL[0]), float(comL[1]), float(comL[2]))

    inertiaL = bodyNode.getElementsByTagName('Inertia')[0].firstChild.data.split()
    inertiaLF = [float(i) for i in inertiaL]
    inertia = toEigen(np.matrix(inertiaLF).reshape((3, 3)))

    obj = []
    for f in bodyNode.getElementsByTagName('File'):
      file = f.firstChild.data
      filename, fileext = os.path.splitext(file)
      filename = os.path.basename(filename)
      obj.append(objpath + filename + '.obj')

    objFiles[id] = obj

    # Transform rotational inertia to body origin
    I_o = sva.PTransform(com).dualMul(sva.RBInertia(mass, Vector3d.Zero(), inertia))

    body = rbd.Body(mass, com, I_o.inertia(), id, label)
    mbg.addBody(body)



  for jointNode in dom.getElementsByTagName('Joint'):
    id = int(jointNode.getAttribute('id'))
    typeN = jointNode.getAttribute('type')
    axisN = jointNode.getAttribute('axis')
    innerId = int(jointNode.getAttribute('innerId'))
    outerId = int(jointNode.getAttribute('outerId'))

    label = jointNode.getElementsByTagName('Label')[0].firstChild.data
    staticL = jointNode.getElementsByTagName('StaticParameters')[0].firstChild.data.split()
    staticLF = [float(i) for i in staticL]
    staticT = Vector3d(staticLF[0], staticLF[1], staticLF[2])
    staticRR = np.deg2rad([staticLF[3], staticLF[4], staticLF[5]])
    staticR = sva.RotZ(staticRR[2])*sva.RotY(staticRR[1])*sva.RotZ(staticRR[0])
    # todo : check the rotation part
    static = sva.PTransform(staticR, staticT)

    type = jointType(typeN, axisN)

    joint = rbd.Joint(type, True, id, label)
    mbg.addJoint(joint)

    mbg.linkBodies(innerId, static, outerId, sva.PTransform.Identity(), id)


  mb = mbg.makeMultiBody(0, isFixed)

  mbc = rbd.MultiBodyConfig(mb)

  zeroParam = []
  zeroDoF = []

  for i in range(mb.nrJoints()):
    zeroParam.append([0.]*mb.joint(i).params())
    zeroDoF.append([0.]*mb.joint(i).dof())

  # identity quaternion
  if not isFixed:
    zeroParam[0][0] = 1.

  mbc.q = zeroParam
  mbc.alpha = zeroDoF
  mbc.alphaD = zeroDoF
  mbc.jointTorque = zeroDoF

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*mb.nrBodies()

  return mb, mbc, mbg, objFiles


