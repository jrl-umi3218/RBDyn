import math

import os.path
from xml.dom.minidom import parse, Document

import numpy as np

from eigen3 import Vector3d, Vector6d, toEigen, toNumpy

import spacevecalg as sva
import rbdyn as rbd



def rotToRPY(r):
  beta = math.atan2(-r[2,0], np.sqrt(r[0,0]**2 + r[1,0]**2))
  if np.allclose(beta, np.pi/2., 1e-6):
    alpha = 0.;
    gamma = math.atan2(r[0,1], r[1,1])
  elif np.allclose(beta, -np.pi/2., 1e-6):
    alpha = 0;
    gamma = -math.atan2(r[0,1], r[1,1])
  else:
    alpha = math.atan2(r[1,0]/np.cos(beta), r[0,0]/np.cos(beta));
    gamma = math.atan2(r[2,1]/np.cos(beta), r[2,2]/np.cos(beta));

  # rpy
  return gamma, beta, alpha



def from_jointType(type, axis):
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


def to_jointType(jointDom, joint):
  def setTypeAxis(type, axis):
    axis = tuple(np.array(axis).flatten())
    jointDom.setAttribute('type', type)
    jointDom.setAttribute('axis3d', '%s %s %s' % axis)

  if joint.type() == rbd.Joint.Rev:
    axis = toNumpy(joint.motionSubspace())[:3]
    setTypeAxis('revolute', axis)
  elif joint.type() == rbd.Joint.Prism:
    axis = toNumpy(joint.motionSubspace())[3:]
    setTypeAxis('prismatic', axis)
  elif joint.type() == rbd.Joint.Spherical:
    jointDom.setAttribute('type', 'spherical')
  elif joint.type() == rbd.Joint.Fixed:
    jointDom.setAttribute('type', 'fixed')


def to_amelif(mb, bound, mesh, scale_xyz=(1., 1., 1.)):
  doc = Document()

  mbDom = doc.createElement('MultiBody')
  doc.appendChild(mbDom)

  rootDom = doc.createElement('Root')
  rootDom.setAttribute('id', str(mb.body(0).id()))
  mbDom.appendChild(rootDom)

  bodiesDom = doc.createElement('Bodies')
  mbDom.appendChild(bodiesDom)

  def addTextElement(name, text, parent):
    dom = doc.createElement(name)
    t = doc.createTextNode(text)
    dom.appendChild(t)
    parent.appendChild(dom)
    return dom

  for b in mb.bodies():
    bodyDom = doc.createElement('Body')
    bodyDom.setAttribute('id', str(b.id()))
    bodyDom.setAttribute('scale', " ".join(["%s"]*3) % scale_xyz)
    bodiesDom.appendChild(bodyDom)

    mass = b.inertia().mass()

    addTextElement('Label', b.name(), bodyDom)
    addTextElement('Mass', str(mass), bodyDom)

    com = b.inertia().momentum()/mass
    addTextElement('CoM', '%s %s %s' % tuple(com), bodyDom)

    # Transform rotational inertia to com origin
    # I_c = I_o - mc_x*c_x^T
    I_o = b.inertia().inertia()
    I_c = I_o - sva.vector3ToCrossMatrix(mass*com)*sva.vector3ToCrossMatrix(com).transpose()

    addTextElement('Inertia', ' '.join(['%s']*9) % tuple(I_c), bodyDom)
    for f in mesh[b.id()]:
      addTextElement('File', f, bodyDom)


  jointsDom = doc.createElement('Joints')
  mbDom.appendChild(jointsDom)

  for i, j in enumerate(list(mb.joints())[1:], 1):
    jointDom = doc.createElement('Joint')
    jointDom.setAttribute('id', str(j.id()))
    jointDom.setAttribute('innerId', str(mb.body(mb.predecessor(i)).id()))
    jointDom.setAttribute('outerId', str(mb.body(mb.successor(i)).id()))
    to_jointType(jointDom, j)
    jointsDom.appendChild(jointDom)

    addTextElement('Label', j.name(), jointDom)

    if bound.has_key(j.id()):
      addTextElement('TorqueLimit', str(abs(bound[j.id()][2])), jointDom)
      addTextElement('SpeedLimit', str(np.rad2deg(bound[j.id()][3])), jointDom)

      addTextElement('PositionMin', str(np.rad2deg(bound[j.id()][0])), jointDom)
      addTextElement('PositionMax', str(np.rad2deg(bound[j.id()][1])), jointDom)

    t = mb.transform(i)
    trans = t.translation()
    rot = toNumpy(t.rotation()).T

    r,p,y = np.rad2deg(rotToRPY(rot))
    stp = (trans[0], trans[1], trans[2], r, p, y)

    addTextElement('StaticParameters', ' '.join(['%s']*6) % stp, jointDom)

  return doc



def from_amelif(file, isFixed):
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
    I_c = toEigen(np.matrix(inertiaLF).reshape((3, 3)))

    obj = []
    for f in bodyNode.getElementsByTagName('File'):
      file = f.firstChild.data
      filename, fileext = os.path.splitext(file)
      filename = os.path.basename(filename)
      obj.append(objpath + filename + '.obj')

    objFiles[id] = obj

    # Transform rotational inertia to body origin
    # I_o = I_c + mc_x*c_x^T
    I_o = I_c + sva.vector3ToCrossMatrix(mass*com)*sva.vector3ToCrossMatrix(com).transpose()

    body = rbd.Body(mass, com, I_o, id, label)
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
    staticR = sva.RotZ(staticRR[2])*sva.RotY(staticRR[1])*sva.RotX(staticRR[0])
    # todo : check the rotation part
    static = sva.PTransform(staticR, staticT)

    type = from_jointType(typeN, axisN)

    joint = rbd.Joint(type, True, id, label)
    mbg.addJoint(joint)

    mbg.linkBodies(innerId, static, outerId, sva.PTransform.Identity(), id)


  mb = mbg.makeMultiBody(0, isFixed)

  mbc = rbd.MultiBodyConfig(mb)
  mbc.zero(mb)

  return mb, mbc, mbg, objFiles


