import pybullet as p
import time
import math
import numpy as np
import pybullet_data
path = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
cubeId = p.loadURDF(path+"TwoLink.urdf", 0, 0, 5)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
cid = p.createConstraint(cubeId, -1, -1, -1, p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 1])
print(cid)
print(p.getConstraintUniqueId(0))
a = -math.pi
pos = np.linspace([0, 0, 1],[0, 0, 10], 5000)
# while True:
# #   a = a + 0.01
# #   if (a > math.pi):
# #     a = -math.pi
#   time.sleep(.01)
#   p.setGravity(0, 0, -10)
#   pivot = [a, 0, 1]
#   orn = p.getQuaternionFromEuler([a, 0, 0])
# #   p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)
# # p.removeConstraint(cid)

for i in range(len(pos)):
  time.sleep(.01)
  p.setGravity(0, 0, -10)
  pivot = pos[i]
  # orn = p.getQuaternionFromEuler([pos[i][0], 0, 0])
  p.changeConstraint(cid, pivot)
