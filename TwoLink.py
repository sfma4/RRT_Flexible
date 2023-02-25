import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import math
import time
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

# boxId = p.loadURDF(path+"TwoLink_2.urdf", basePosition=[0, 0, 2],
#                    baseOrientation=p.getQuaternionFromEuler([math.pi/2, 0, 0]), useFixedBase=False)
# objId = p.loadURDF(path+"cube.urdf", basePosition=[2, 0, 4],
#                    useFixedBase=True, globalScaling=2)
boxId = p.loadURDF(path+"TwoLink_4.urdf", basePosition=[2, 0, 2],
                   baseOrientation=p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0]), useFixedBase=False)


objId = p.loadURDF(path+"cube.urdf", basePosition=[7.5, 0, 2.5],
                   useFixedBase=True, globalScaling=5)

cubeStartPos = [-2.15,0,.75]
cubeStartPos2 = [0,0,1.4]
cubeStartPos3 = [2.15,0,.75]

PulleyStartOrientation = p.getQuaternionFromEuler([1.570796, 0, 0])
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
cubeStartOrientation2 = p.getQuaternionFromEuler([0,-1.570796,0])
path2 = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\data\\"
# pendulum = p.loadURDF(path2 + "Pendulum_Tendon_1_Cart_Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
# boxId = p.loadURDF(path2 + "Pendulum_Tendon_1_Cart_Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)

# p.setTimeStep(0.001)
# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
orientation = p.getQuaternionFromEuler([0,np.pi/2,0])
# cid = p.createConstraint(boxId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
# print(cid)
# print(p.getConstraintUniqueId(0))
a = -math.pi
# pos1 = np.linspace([0, 0, 5], [0, 3, 5], 100)
# pos2 = np.linspace([0, 2.999, 5],[5, 2, 5], 100)
# pos3 = np.linspace([5, 2, 5],[5, 2.999, 5], 100)
t = 100 # path steps
pos1 = np.linspace([0, 0, 2], [0, 5, 5], t)
pos2 = np.linspace([0, 5, 5],[0, 0, -1], t)
pos = np.concatenate((pos1, pos2))
o1 = p.getQuaternionFromEuler([0, 0, 0])
o2 = p.getQuaternionFromEuler([0, 0, np.pi/2])

orn1 = np.linspace(o1, o2, t)
orn2 = np.linspace(o2, o1, t)
orn = np.concatenate((orn1, orn2))
# logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")


available_joints_indexes = [i for i in range(p.getNumJoints(boxId)) if p.getJointInfo(boxId, i)[2] != p.JOINT_FIXED]

print('available joints index', [p.getJointInfo(boxId, i)[1] for i in available_joints_indexes])
# while True:
#     p.stepSimulation()
#     # there can be some artifacts in the visualizer window,
#     # due to reading of deformable vertices in the renderer,
#     # while the simulators updates the same vertices
#     # it can be avoided using
#     # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
#     # but then things go slower
#     p.setGravity(0, 0, -10)
#     # sleep(1./240.)
#     # result = p.getContactPoints(planeId, boxId)
#     # print(p.getLinkState(boxId, 4))
#     result = p.getBasePositionAndOrientation(boxId)
#     # print(result)
# for i in range(len(pos)):
#   time.sleep(.001)
#   p.setGravity(0, 0, -10)
#   pivot = pos[i]
#   # orn = p.getQuaternionFromEuler([0,np.pi/2,0])
#   p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn[i])
wheel_joints_indexes = [i for i in available_joints_indexes if "joint_" in str(p.getJointInfo(boxId, i)[1])]
# wheel_joints_indexes = available_joints_indexes[:2]
print('wheel_joints_indexes', wheel_joints_indexes)
target_v = 10  # motor angular speed rad/s
# target_v = [target_v, target_v]
max_force = 100 # max force exerted by motor
for i in range(len(pos)):
    p.stepSimulation()
    p.setGravity(0, 0, -10)
    # p.setJointMotorControl2(
    #     boxId,
    #     4,
    #     p.VELOCITY_CONTROL,
    #     force=100
    # )
    p.setGravity(0, 0, -10)
    # p.enableJointForceTorqueSensor(boxId, 4)
    p.setJointMotorControl2(
        boxId,
        0,
        p.POSITION_CONTROL,
        pos[:, 0][i],
        target_v,
        max_force
    )
    p.setJointMotorControl2(
        boxId,
        2,
        p.POSITION_CONTROL,
        pos[:, 1][i],
        target_v,
        max_force
    )

    p.setJointMotorControl2(
        boxId,
        0,
        p.POSITION_CONTROL,
        pos[i][2],
        target_v,
        max_force
    )

    print(p.getJointState(boxId, 1))

    time.sleep(0.0001)
p.resetSimulation()
# p.stopStateLogging(logId)