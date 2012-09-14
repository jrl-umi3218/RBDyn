import time
import pickle
from multiprocessing import Process, Array, Value, Condition

import numpy as np

from mayavi.tools.animator import Animator

from eigen3 import *
import rbdyn as rbd
import spacevecalg as sva

import tasks
import scd

from graph.multibody import GraphicMultiBody
from graph.geometry import MeshGeometry

import robots.little_human as hoap

import nethoap
import kinect

staticRad = 0.021
staticObs = Vector3d(0.151, 0.151, 0.351)

started = Value('b', False)
qDes = Array('d', 21)
qReal = Array('d', 21)
startCond = Condition()

qDesData = np.frombuffer(qDes.get_obj())
qRealData = np.frombuffer(qReal.get_obj())

qDesData[:] = np.mat(np.zeros((21,)))
qRealData[:] = np.mat(np.zeros((21,)))

def kinectRHandPos():
  kinPos = np.mat(kinect.rHandPos[:])/1000. - np.mat([-0.14, -0.21, 2.66 + 0.15])
  kinRot = np.mat([[0., 0., -1.],
                   [1., 0., 0.],
                   [0., 1., 0.]])
  kinPos = kinRot*kinPos.T

  return kinPos


def paramToPython(param):
  res = list(param)
  for i in range(len(res)):
    res[i] = list(res[i])
  return res


stpbv = {}
def initSTPBV(stpbvFiles):
  for id, file in stpbvFiles.items():
    stpbv[id] = scd.STPBV(file)



def computeTransform(mbOrig, mbTrans):
  """
  Compute the transformations to apply a mesh in mbOrig coordinate
  in mbTrans coordiante.
  """
  trans = [sva.PTransform.Identity()]*mbTrans.nrBodies()

  origRootId = mbOrig.body(0).id()

  curTransId = mbTrans.body(0).id()
  curBody = 0
  curJoint = 1

  while curTransId != origRootId:
    trans[curBody] = mbTrans.transform(curJoint)

    curBody += 1
    curJoint += 1
    curTransId = mbTrans.body(curBody).id()


  if curJoint > 1:
    bodyId = mbTrans.body(mbTrans.predecessor(curJoint - 1)).id()
    posInOrig = mbOrig.bodyIndexById(bodyId)
    # index of support joint and his body is the same
    trans[curBody] = mbOrig.transform(posInOrig).inv()

  return trans



# create real robot
mb, mbc, mbg = hoap.robot()
bound = hoap.bound(mb)
objfile, stpbvfiles, convexfiles = hoap.ressources('../../robots/hoap_3/')

initSTPBV(stpbvfiles)



class littleHuman(object):
  def __init__(self, bodyId, transform):
    self.mb = mbg.makeMultiBody(bodyId, True, transform)
    self.mbc = rbd.MultiBodyConfig(self.mb)

    self.toLocal = rbd.ConfigConverter(mb, self.mb)
    self.toReal = rbd.ConfigConverter(self.mb, mb)

    self.toLocal.convert(mbc, self.mbc)

    self.transform = computeTransform(mb, self.mb)

    self.transformById = {}
    for i, t in enumerate(self.transform):
      self.transformById[self.mb.body(i).id()] = t

    self.bound = hoap.bound(self.mb)



# create Right Foot and Left Foot robot
r = np.mat([[0., 0., 1.],
            [0., -1., 0.],
            [1., 0., 0.]])
tr = sva.PTransform(toEigen(r))

robotRF = littleHuman(6, tr)
robotLF = littleHuman(16, tr)




class Controller(object):
  def __init__(self):
    HOST = '10.59.145.197'
    PORT = 55000

    # self.robot = nethoap.NetHoap3(HOST, PORT, mb)
    self.robot = nethoap.FakeHoap3(HOST, PORT, mb)

    self.robotPlan = robotRF

    self.mbcReal = rbd.MultiBodyConfig(mbc)
    self.mbcControl = rbd.MultiBodyConfig(mbc)

    self.solver = tasks.qp.QPSolver()


  def begin(self):
    self.robot.open()
    self.initTask()


  def end(self):
    self.robot.close()


  def initTask(self):
    mbP = self.robotPlan.mb
    mbcP = self.robotPlan.mbc

    # take current q
    q, alpha = self.robot.sensor()
    q = toEigenX(q)
    alpha = toEigenX(alpha)

    self.mbcReal.q = rbd.vectorToParam(mb, q)
    # self.mbcReal.alpha = rbd.vectorToDof(mb, alpha)

    self.robotPlan.toLocal.convert(self.mbcReal, mbcP);

    # update desired config
    rbd.forwardKinematics(mbP, mbcP)
    rbd.forwardVelocity(mbP, mbcP)

    bodyPosW = list(mbcP.bodyPosW)

    # compute CoM pos objective
    com = rbd.computeCoM(mbP, mbcP)
    comObj = Vector3d(com[0], com[1], com[2])
    # comObj = Vector3d(self.lFootPos[0], self.lFootPos[1], comObj[2])
    self.com = 'e'

    # PostureTask
    self.postureTask = tasks.qp.PostureTask(mbP, paramToPython(mbcP.q), 3., 10.)

    # CoMTask
    self.comTask = tasks.qp.CoMTask(mbP, comObj)
    self.comTaskSP = tasks.qp.SetPointTask(mbP, self.comTask, 4., 5000.)

    # HandsTask
    lhandI = mbP.bodyIndexById(20)
    self.lhandPTask = tasks.qp.PositionTask(mbP, 20, Vector3d(0.22, 0.05 - 0.15, 0.37 - 0.03),
                                            Vector3d(0., 0.16, 0.))
    rhandI = mbP.bodyIndexById(10)
    self.rhandPTask = tasks.qp.PositionTask(mbP, 10, Vector3d(0.22, 0.05 + 0.15, 0.37 + 0.03),
                                            Vector3d(0., 0.16, 0.))
    self.lhandPTaskSP = tasks.qp.SetPointTask(mbP, self.lhandPTask, 4., 1000.)
    self.rhandPTaskSP = tasks.qp.SetPointTask(mbP, self.rhandPTask, 4., 1000.)
    self.handS = 'i'

    # add tasks
    self.solver.addTask(self.comTaskSP)
    self.solver.addTask(self.lhandPTaskSP)
    self.solver.addTask(self.rhandPTaskSP)
    self.solver.addTask(self.postureTask)

    cont = []

    c = tasks.qp.Contact()
    c.bodyId = 16
    c.points = [Vector3d.Zero()]
    c.normals = [Vector3d.UnitZ()]
    cont.append(c)

    c = tasks.qp.Contact()
    c.bodyId = 6
    c.points = [Vector3d.Zero()]
    c.normals = [Vector3d.UnitZ()]
    cont.append(c)



    # MotionConstraint
    self.motionConstr = tasks.qp.MotionConstr(mbP)
    self.solver.addEqualityConstraint(self.motionConstr)
    self.solver.addBoundConstraint(self.motionConstr)
    self.solver.addConstraint(self.motionConstr)

    # ContactConstraint
    self.contactConstr = tasks.qp.ContactAccConstr(mbP)

    self.solver.addEqualityConstraint(self.contactConstr)
    self.solver.addConstraint(self.contactConstr)

    # JointLimitsConstraint
    lBound = paramToPython(self.robotPlan.bound[0])
    uBound = paramToPython(self.robotPlan.bound[1])
    self.jointLimitsConstr = tasks.qp.JointLimitsConstr(mbP, lBound, uBound, 0.005)
    self.solver.addBoundConstraint(self.jointLimitsConstr)
    self.solver.addConstraint(self.jointLimitsConstr)

    # SelfCollisionConstraint
    self.selfCollConstr = tasks.qp.SelfCollisionConstr(mbP, 0.005)
    def addColl(id1, id2, di, ds, damp):
      self.selfCollConstr.addCollision(mbP, id1, stpbv[id1], self.robotPlan.transformById[id1],
                                       id2, stpbv[id2], self.robotPlan.transformById[id2], di, ds, damp)

    addColl(10, 20, 0.1, 0.01, 0.5)
    addColl(10, 21, 0.02, 0.005, 0.05)
    addColl(20, 21, 0.02, 0.005, 0.05)
    addColl(13, 3, 0.01, 0.001, 0.05)

    self.pair = scd.CD_Pair(stpbv[10], stpbv[20])

    self.solver.addInequalityConstraint(self.selfCollConstr)
    self.solver.addConstraint(self.selfCollConstr)

    # StaticEnvCollisionConstraint
    self.seCollConstr = tasks.qp.StaticEnvCollisionConstr(mbP, 0.005)
    staticSphere = scd.Sphere(staticRad)
    staticSphere.transform(sva.PTransform(staticObs))
    self.seCollConstr.addCollision(mbP, 20, stpbv[20], self.robotPlan.transformById[20],
                                   0, staticSphere, 0.1, 0.01, 0.5)
    self.seCollConstr.addCollision(mbP, 19, stpbv[19], self.robotPlan.transformById[19],
                                   0, staticSphere, 0.1, 0.01, 0.5)
    self.seCollConstr.addCollision(mbP, 18, stpbv[18], self.robotPlan.transformById[18],
                                   0, staticSphere, 0.1, 0.01, 0.5)
    self.ss = staticSphere

    self.solver.addInequalityConstraint(self.seCollConstr)
    self.solver.addConstraint(self.seCollConstr)

    self.solver.nrVars(mbP, cont)
    self.solver.updateEqConstrSize()
    self.solver.updateInEqConstrSize()


  def run(self):
    self.ss.transform(sva.PTransform(toEigen(kinectRHandPos())))
    mbP = self.robotPlan.mb
    mbcP = self.robotPlan.mbc

    # compute next desired position
    self.solver.update(mbP, mbcP)
    rbd.eulerIntegration(mbP, mbcP, 0.005)


    if np.linalg.norm(toNumpy(self.comTask.eval())) < 0.025:
      if self.com == 'l':
        print 'left com'
        obj = self.comTask.com()
        obj[0] = self.rFootPos[0]
        obj[1] = self.rFootPos[1]
        self.comTask.com(obj)
        self.com = 'r'
      elif self.com == 'r':
        print 'right com'
        obj = self.comTask.com()
        obj[0] = self.lFootPos[0]
        obj[1] = self.lFootPos[1]
        self.comTask.com(obj)
        self.com = 'l'


    dist = np.sqrt(self.pair.distance())
    if dist < 0.005:
      print np.sqrt(self.pair.distance())

    if self.handS == 'i' and np.linalg.norm(toNumpy(self.lhandPTask.eval())) < 0.18\
       and np.linalg.norm(toNumpy(self.rhandPTask.eval())) < 0.18:
      # print 'init'
      pl = self.lhandPTask.position()
      pr = self.rhandPTask.position()
      pr[0] = pr[0] - 0.05
      pl[0] = pl[0] + 0.05
      pr[2] = pr[2] - 0.2
      pl[2] = pl[2] + 0.2
      self.lhandPTask.position(pl)
      self.rhandPTask.position(pr)
      self.handS = 'r'
    if self.handS == 'r' and np.linalg.norm(toNumpy(self.lhandPTask.eval())) < 0.16\
       and np.linalg.norm(toNumpy(self.rhandPTask.eval())) < 0.16:
      # print 'switch'
      pl = self.lhandPTask.position()
      pr = self.rhandPTask.position()
      t = pr[2]
      pr[2] = pl[2]
      pl[2] = t
      t = pr[0]
      pr[0] = pl[0]
      pl[0] = t
      self.lhandPTask.position(pl)
      self.rhandPTask.position(pr)


    self.robotPlan.toReal.convert(mbcP, self.mbcControl);

    # control the robot
    self.robot.control(toNumpy(rbd.paramToVector(mb, self.mbcControl.q)),
                       toNumpy(rbd.dofToVector(mb, self.mbcControl.alpha)))

    # read sensors
    q, alpha = self.robot.sensor()
    q = toEigenX(q)
    alpha = toEigenX(alpha)

    # update real config
    self.mbcReal.q = rbd.vectorToParam(mb, q)
    # self.mbcReal.alpha = rbd.vectorToDof(mb, alpha)

    # update desired config
    # self.robotPlan.toLocal(mbcReal, mbcP);
    rbd.forwardKinematics(mbP, mbcP)
    rbd.forwardVelocity(mbP, mbcP)



def controllerProcess():
  cont = Controller()

  startCond.acquire()
  if not started.value:
    startCond.wait()
  startCond.release()

  cont.begin()

  des = []
  real = []

  while started.value:
    cmpTimeDeb = time.time()
    cont.run()

    realParamInLocal = cont.robotPlan.toLocal.convertJoint(cont.mbcReal.q)

    desQ = toNumpy(rbd.paramToVector(cont.robotPlan.mb, cont.robotPlan.mbc.q)).T
    realQ = toNumpy(rbd.paramToVector(cont.robotPlan.mb, realParamInLocal)).T

    des.append(desQ)
    real.append(realQ)

    if qDes.get_lock().acquire(False):
      try:
        qDesData = np.frombuffer(qDes.get_obj())
        qRealData = np.frombuffer(qReal.get_obj())

        qDesData[:] = desQ
        qRealData[:] = realQ
      finally:
        qDes.get_lock().release()

    cmpTimeEnd = time.time()
    ellapsed = cmpTimeEnd - cmpTimeDeb
    ellapsed = min(0.005, ellapsed)

    time.sleep(0.005 - ellapsed)

  cont.end()

  pkl_file = open('data.pkl', 'wb')
  pickle.dump((des, real), pkl_file)
  pkl_file.close()



def replayProcess():
  pkl_file = open('data.pkl', 'rb')
  des, real = pickle.load(pkl_file)

  for d, r in zip(des, real):
    with qDes.get_lock():
      qDesData = np.frombuffer(qDes.get_obj())
      qRealData = np.frombuffer(qReal.get_obj())

      qDesData[:] = d
      qRealData[:] = r

    time.sleep(0.005)

  pkl_file.close()



class Displayer(object):
  def __init__(self):
    self.robotPlan = robotRF

    mbP = self.robotPlan.mb
    mbcP = self.robotPlan.mbc

    geomReal = MeshGeometry(mbP, objfile, 1.)
    geomDes = MeshGeometry(mbP, objfile, 1.)

    self.mbcDes = rbd.MultiBodyConfig(mbcP)
    self.mbcReal = rbd.MultiBodyConfig(mbcP)

    rbd.forwardKinematics(mbP, self.mbcDes)
    rbd.forwardKinematics(mbP, self.mbcReal)

    self.graphDes = GraphicMultiBody(mbP, geomDes,
                                     sva.PTransform(Vector3d(0., 0.5, 0.)),
                                     bodyBase=self.robotPlan.transform)
    self.graphReal = GraphicMultiBody(mbP, geomReal, bodyBase=self.robotPlan.transform)

    self.graphDes.draw(mbP, self.mbcDes)
    self.graphReal.draw(mbP, self.mbcReal)

    self.obj = sphere(staticRad)
    self.obj.position = list(kinect.rHandPos[:])

    self.graphReal.render()


  def run(self):
    mbP = self.robotPlan.mb

    qDesVec = np.mat(np.zeros((21,1)))
    qRealVec = np.mat(np.zeros((21,1)))

    pair = [(20, 10)]
    cdPair = []

    for p in pair:
      b1 = p[0]
      b2 = p[1]
      l = line()
      s1 = sphere(0.005)
      s2 = sphere(0.005)
      pa = scd.CD_Pair(stpbv[b1], stpbv[b2])
      cdPair.append((pa, l, s1, s2, b1, b2))

    while True:
      with qDes.get_lock():
        qDesData = np.frombuffer(qDes.get_obj())
        qRealData = np.frombuffer(qReal.get_obj())
        qDesVec[:] = np.mat(qDesData).T
        qRealVec[:] = np.mat(qRealData).T


      self.obj.position = kinectRHandPos().T.tolist()[0]
      #print np.array(kinect.rHandPos[:])/1000.


      self.mbcDes.q = rbd.vectorToParam(mbP, toEigenX(qDesVec))
      self.mbcReal.q = rbd.vectorToParam(mbP, toEigenX(qRealVec))

      rbd.forwardKinematics(mbP, self.mbcDes)
      rbd.forwardKinematics(mbP, self.mbcReal)

      self.graphDes.draw(mbP, self.mbcDes)
      self.graphReal.draw(mbP, self.mbcReal)


      p1 = Vector3d()
      p2 = Vector3d()
      for pa, l, s1, s2, b1, b2 in cdPair:
        ib1 = mbP.bodyIndexById(b1)
        ib2 = mbP.bodyIndexById(b2)
        stpbv[b1].transform(self.robotPlan.transformById[ib1]*list(self.mbcReal.bodyPosW)[ib1])
        stpbv[b2].transform(self.robotPlan.transformById[ib2]*list(self.mbcReal.bodyPosW)[ib2])

        d1 = np.sqrt(pa.distance(p1, p2))
        lp1 = list(p1)
        lp2 = list(p2)
        l.point1 = lp1
        l.point2 = lp2
        s1.position = lp1
        s2.position = lp2

        stpbv[b1].transform(self.robotPlan.transformById[ib1]*list(self.mbcDes.bodyPosW)[ib1])
        stpbv[b2].transform(self.robotPlan.transformById[ib2]*list(self.mbcDes.bodyPosW)[ib2])
        d2 = np.sqrt(pa.distance())


      self.graphReal.render()

      yield


def line():
  from tvtk.api import tvtk
  from mayavi import mlab

  l = tvtk.LineSource()
  m = tvtk.PolyDataMapper(input=l.output)
  a = tvtk.Actor(mapper=m)
  mlab.gcf().scene.add_actor(a)

  return l



def sphere(r=0.03):
  from tvtk.api import tvtk
  from mayavi import mlab

  s = tvtk.SphereSource(radius=r)
  m = tvtk.PolyDataMapper(input=s.output)
  a = tvtk.Actor(mapper=m)
  mlab.gcf().scene.add_actor(a)

  return a



if __name__ == '__main__':
  contProc = Process(target=controllerProcess)
  repProc = Process(target=replayProcess)
  kinectProc = Process(target=kinect.kinect)

  graph = Displayer()

  a = Animator(33, graph.run().next)
  a.timer.Stop()

  def kinectV():
    kinectProc.start()
    a.timer.Start()

  def kinectVS():
    a.timer.Stop()
    kinectProc.terminate()
    kinectProc.join()


  def replay():
    repProc.start()
    a.timer.Start()

  def replayS():
    a.timer.Stop()
    repProc.terminate()
    repProc.join()


  def start():
    kinectProc.start()
    contProc.start()

    startCond.acquire()
    started.value = True
    startCond.notify()
    startCond.release()
    a.timer.Start()

  def stop():
    a.timer.Stop()
    started.value = False
    kinectProc.terminate()
    contProc.join()
    kinectProc.join()


  #a.edit_traits()

