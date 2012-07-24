import numpy as np

from eigen3 import toEigen
import spacevecalg as sva

def dhToTransform(a, alpha, d, theta):
  t = np.mat([a*np.cos(theta), a*np.sin(theta), d]).T
  r = np.mat([[np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta)],
              [np.sin(theta), np.cos(alpha)*np.cos(theta), -np.cos(theta)*np.sin(alpha)],
              [0., np.sin(alpha), np.cos(alpha)]])

  return sva.PTransform(toEigen(r), toEigen(t))


