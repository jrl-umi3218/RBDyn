import socket
import struct

import numpy as np


thetaToPulse = np.array([209, 209, -209, 209, 209, -209, 209, 209, -209, -209,
                         209, 209, 209, -209, -209, -209, -209, 209, -209, 209,
                         209])


pulseToTheta = 1./thetaToPulse


sensorStruct = struct.Struct('21h21h')
controlStruct = struct.Struct('21h')


hoap_init = np.array([0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
                      0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800,
                      418])



class FakeHoap3(object):
  def __init__(self, host, port, mb):
    # compute index to make hoap -> rbdyn and rbdyn -> hoap
    self.index = np.array([mb.jointPosInParam(mb.jointIndexById(i)) for i in range(1, 22)])

    self.curQ = hoap_init


  def __enter__(self):
    pass


  def __exit__(self):
    pass


  def open(self):
    pass


  def close(self):
    pass


  def control(self, q):
    q = np.array(q).reshape((21,))
    qHoap = np.rad2deg(q[self.index])*thetaToPulse
    self.curQ = np.array(qHoap, dtype='short')


  def sensor(self):
    q = np.deg2rad(self.curQ*pulseToTheta)
    qn = q.copy()
    qn[self.index] = q
    qn = np.mat(qn).T

    return qn



class NetHoap3(object):
  def __init__(self, host, port, mb):
    self.host = (host, port)

    # compute index to make hoap -> rbdyn and rbdyn -> hoap
    self.index = np.array([mb.jointPosInParam(mb.jointIndexById(i)) for i in range(1, 22)])

    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


  def __enter__(self):
    self.open()


  def __exit__(self):
    self.close()


  def open(self):
    self.sock.connect(self.host)
    print 'Connected'


  def close(self):
    self.sock.close()
    print 'Disconnected'


  def control(self, q):
    qHoap = np.rad2deg(np.array(q)[self.index].T)*thetaToPulse
    qnHoap = np.array(qHoap, dtype='short').T

    self.sock.sendall(controlStruct.pack(*qnHoap))


  def sensor(self):
    data = self.sock.recv(sensorStruct.size)
    if data == None or len(data) != sensorStruct.size:
      raise RuntimeError('Connexion broken')

    pyData = sensorStruct.unpack(data)

    q = np.deg2rad(np.array(pyData[:21])*pulseToTheta)
    qn = q.copy()
    qn[self.index] = q
    qn = np.mat(qn).T

    return qn

