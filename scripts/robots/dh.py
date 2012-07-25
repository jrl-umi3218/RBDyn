from eigen3 import Vector3d
import spacevecalg as sva

def dhToTransform(a, alpha, d, theta):
  rx = sva.PTransform(sva.RotX(alpha))
  tx = sva.PTransform(Vector3d.UnitX()*a)
  rz = sva.PTransform(sva.RotZ(theta))
  tz = sva.PTransform(Vector3d.UnitZ()*d)

  return tz*rz*tx*rx
