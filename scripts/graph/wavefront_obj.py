import numpy as np

def readObj(filename):
  vert = []
  face = []

  with open(filename, 'r') as file:
    for line in file:
      l = line.split()
      if len(l) > 0:
        if l[0] == 'v':
          vert.append(l[1:])
        elif l[0] == 'f':
          face.append(l[1:])

  return (np.array(vert, np.double), np.array(face, np.int))

