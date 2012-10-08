import os

import robots.little_human as hoap

from robots.urdf import to_urdf

if __name__ == '__main__':
  mb, mbc, mbg = hoap.robot()
  boundVec = hoap.bound(mb)
  objfile, stpbvfiles, convexfiles = hoap.ressources('')

  boundDict = dict([(mb.joint(i).id(), (l[0], u[0])) for i, (l, u) in enumerate(zip(*boundVec)) if len(l) > 0])

  stlfile = dict([(id, 'hoap3_description' + os.sep + 'meshes' + os.sep + file[0].split(os.sep)[-1].replace('.obj', '.stl')) for id, file in objfile.iteritems()])

  doc = to_urdf('hoap3', mb, boundDict, stlfile)
  print doc.toprettyxml(indent='  ')

