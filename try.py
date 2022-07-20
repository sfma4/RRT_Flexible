import pybullet as p
from time import sleep
import pybullet_data
import os
# sys.path.append("C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\data")
path = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\"
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeOrn = [0, 0, 0, 1]  # p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF(path+"plane.urdf", [0, 0, -2], planeOrn)

# boxId = p.loadURDF(path+"TwoLink.urdf", [0, 3, 2], useMaximalCoordinates=True)
# boxId = p.loadURDF(path+"TwoLink.urdf",  useFixedBase=True)

boxId = p.loadURDF(path+"franka_panda/panda_flexible.urdf", useFixedBase=True)

# ballId = p.loadSoftBody(path+"ball.obj", simFileName=path+"ball.vtk", basePosition=[0, 0, -1], scale=0.5, mass=4, useNeoHookean=1,
#                         NeoHookeanMu=400, NeoHookeanLambda=600, NeoHookeanDamping=0.001, useSelfCollision=1,
#                         frictionCoeff=.5, collisionMargin=0.001)
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

# logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")

while p.isConnected():
    p.stepSimulation()
    # there can be some artifacts in the visualizer window,
    # due to reading of deformable vertices in the renderer,
    # while the simulators updates the same vertices
    # it can be avoided using
    # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
    # but then things go slower
    p.setGravity(0, 0, -10)
    # sleep(1./240.)
    # result = p.getContactPoints(planeId, boxId)
    result = p.getBasePositionAndOrientation(boxId)
    # print(result)

# p.resetSimulation()
# p.stopStateLogging(logId)
