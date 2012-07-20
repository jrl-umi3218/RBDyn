import os.path

import numpy as np


def readMaterial(filename):
  mat = {}
  curMat = {}

  with open(filename, 'r') as file:
    for line in file:
      l = line.split()
      if len(l) > 0:
        if l[0] == 'newmtl':
          curMat = {}
          mat[l[1]] = curMat
        elif l[0] != '#':
          curMat[l[0]] = np.array(l[1:], np.double)

  return mat




def readObj(filename):
  vert = []
  face = []
  color = []
  mat = {}
  curMat = {}
  curColor = (0., 0., 0.)

  with open(filename, 'r') as file:
    for line in file:
      l = line.split()
      if len(l) > 0:
        if l[0] == 'v':
          vert.append(l[1:])
        elif l[0] == 'f':
          face.append(l[1:])
          color.append(curColor)
        elif l[0] == 'mtllib':
          mat = readMaterial(os.path.dirname(filename) + os.path.sep + l[1])
        elif l[0] == 'usemtl':
          curMat = mat[l[1]]
          curColor = tuple(curMat['Kd']*255.)

  return (np.array(vert, np.double), np.array(face, np.int) - 1, color)

