import math

from xml.dom.minidom import Document

import numpy as np

import spacevecalg as sva
import rbdyn as rbd
from eigen3 import toNumpy



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
  return -gamma, -beta, -alpha



def setJointType(doc, jointDom, joint, bound):
  def addAxis(aStr):
    axis = doc.createElement('axis')
    axis.setAttribute('xyz', aStr)
    jointDom.appendChild(axis)
  def addLimits():
    limit = doc.createElement('limit')
    limit.setAttribute('effort', str(30.))
    limit.setAttribute('velocity', str(1.))
    limit.setAttribute('lower', str(bound[joint.id()][0]))
    limit.setAttribute('upper', str(bound[joint.id()][1]))
    jointDom.appendChild(limit)

  if joint.type() == rbd.Joint.RevX:
    jointDom.setAttribute('type', 'revolute')
    addAxis('1 0 0')
    addLimits()
  elif joint.type() == rbd.Joint.RevY:
    jointDom.setAttribute('type', 'revolute')
    addAxis('0 1 0')
    addLimits()
  elif joint.type() == rbd.Joint.RevZ:
    jointDom.setAttribute('type', 'revolute')
    addAxis('0 0 1')
    addLimits()
  elif joint.type() == rbd.Joint.PrismX:
    jointDom.setAttribute('type', 'prismatique')
    addAxis('1 0 0')
    addLimits()
  elif joint.type() == rbd.Joint.PrismY:
    jointDom.setAttribute('type', 'prismatique')
    addAxis('0 1 0')
    addLimits()
  elif joint.type() == rbd.Joint.PrismZ:
    jointDom.setAttribute('type', 'prismatique')
    addAxis('0 0 1')
    addLimits()
  elif joint.type() == rbd.Joint.Spherical:
    jointDom.setAttribute('type', 'continuous')
  elif joint.type() == rbd.Joint.Free:
    jointDom.setAttribute('type', 'floating')
  elif joint.type() == rbd.Joint.Fixed:
    jointDom.setAttribute('type', 'fixed')


def to_urdf(name, mb, bound, mesh):
  doc = Document()

  robot = doc.createElement('robot')
  robot.setAttribute('name', name)
  doc.appendChild(robot)

  link = doc.createElement('link')
  link.setAttribute('name', 'base_link')
  robot.appendChild(link)

  for b in mb.bodies():
    link = doc.createElement('link')
    link.setAttribute('name', b.name())
    robot.appendChild(link)

    visual = doc.createElement('visual')
    link.appendChild(visual)

    geom = doc.createElement('geometry')
    visual.appendChild(geom)

    meshDom = doc.createElement('mesh')
    meshDom.setAttribute('filename', 'package://%s' % mesh[b.id()])
    geom.appendChild(meshDom)

    material = doc.createElement('material')
    material.setAttribute('name', 'Grey')
    visual.appendChild(material)

    color = doc.createElement('color')
    color.setAttribute('rgba', '0.2 0.2 0.2 1.0')
    material.appendChild(color)


  for i, j in enumerate(mb.joints()):
    joint = doc.createElement('joint')

    joint.setAttribute('name', j.name())
    setJointType(doc, joint, j, bound)

    parentName = 'base_link' if i == 0 else mb.body(mb.predecessor(i)).name()
    parent = doc.createElement('parent')
    parent.setAttribute('link', parentName)
    joint.appendChild(parent)

    child = doc.createElement('child')
    child.setAttribute('link', mb.body(mb.successor(i)).name())
    joint.appendChild(child)

    t = mb.transform(i)
    pos = toNumpy(t.translation())
    r = toNumpy(t.rotation())

    rRot, pRot, yRot = rotToRPY(r)

    rr = sva.PTransform(sva.RotX(rRot))
    pr = sva.PTransform(sva.RotY(pRot))

    r = np.mat([rRot, 0., 0.]).T
    p = toNumpy(rr.rotation()).T*np.mat([0., pRot, 0.]).T
    y = toNumpy((pr*rr).rotation()).T*np.mat([0., 0., yRot]).T
    rpy = r + p + y


    origin = doc.createElement('origin')
    origin.setAttribute('xyz', '%s %s %s' % (pos[0,0], pos[1,0], pos[2,0]))
    origin.setAttribute('rpy', '%s %s %s' % (rpy[0,0], rpy[1,0], rpy[2,0]))
    joint.appendChild(origin)
    # res = yr*pr*rr

    robot.appendChild(joint)

  return doc

