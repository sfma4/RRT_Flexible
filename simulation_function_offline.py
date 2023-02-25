import pybullet as p
from time import sleep
import time
import pybullet_data
import math
import matplotlib.pyplot as plt
import numpy as np
path = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\"
physicsClient = p.connect(p.GUI)
p.resetSimulation()
p.setRealTimeSimulation(0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera( cameraDistance=8.8, cameraYaw=36.0, cameraPitch=-30, cameraTargetPosition=[2.3,2.0,0])

# boxId = p.loadURDF(path+"TwoLink_Chain.urdf", basePosition=[2.75, 0, 2],
#                    baseOrientation=p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0]), useFixedBase=False)
# boxId = p.loadURDF(path+"short_chain.urdf", basePosition=[2.75, 0, 2],
#                    baseOrientation=p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0]), useFixedBase=False)

# box2 = p.loadURDF(path+"TwoLink_4.urdf", basePosition=[5.75, 0, 3],
#                    baseOrientation=p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0]), useFixedBase=False)
boxId = p.loadURDF(path+"TwoLink_4.urdf", basePosition=[2.75, 0, 2],
                   baseOrientation=p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0]), useFixedBase=False)


# objId = p.loadURDF(path+"cube.urdf", basePosition=[7.5, 0, 2.5],
#                    useFixedBase=True, globalScaling=5)
ob2 = p.loadURDF(path + "cube.urdf", basePosition=[8, 0, 5.5],
                 useFixedBase=True, globalScaling=1)
ob3 = p.loadURDF(path + "cube.urdf", basePosition=[7, 0, 6.5],
                 useFixedBase=True, globalScaling=1)

target_v = 3  # motor angular speed rad/s
# target_v = [target_v, target_v]
max_force = 1000  # max force exerted by motor
x_plot, y_plot = [], []
# pos1 = np.linspace([0, 0, 5], [0, 3, 5], 100)
# pos2 = np.linspace([0, 2.999, 5],[5, 2, 5], 100)
# pos3 = np.linspace([5, 2, 5],[5, 2.999, 5], 100)
# pos = np.concatenate((pos1, pos2))
# path = np.concatenate((pos, pos3))
# p.setTimeStep(0.02)
p1 = np.linspace([0, 0], [-3, 0], 100)
print('lenth', len(path))
print('path', path)
force = []
p.resetJointState(boxId, 0, p1[0][0])
p.resetJointState(boxId, 2, p1[0][1])
# p.resetJointState(boxId, 3, p1[0][2])
p.resetJointState(boxId, 3, 0.2)
p.changeDynamics(boxId, 4, lateralFriction=0.2)
p.changeDynamics(boxId, 3, linearDamping=0.0, angularDamping=0)
p.changeDynamics(boxId, 4, lateralFriction=0.25)
print(p.getJointState(boxId,3))
# for i in range(100):
angle = []
f_plot = []
print("joint dynamics", p.getDynamicsInfo(boxId,3))
for i in range(1000):
    p.stepSimulation()
    p.setGravity(0, 0, -30.0)
    x = p.getJointState(boxId, 0)[0]
    y = p.getJointState(boxId, 2)[0]
    f = p.getJointState(boxId, 4)[0]
    # if abs(x - 3) < 0.1 and (y - 3) < 0.1:
    #     print('loop end', 'x =', x, y)
    #     break
    # p.setJointMotorControl2(
    #     boxId,
    #     0,
    #     p.POSITION_CONTROL,
    #     # np.sin(i/50),
    #     0,
    #     target_v,
    #     max_force
    # )
    # p.setJointMotorControl2(
    #     boxId,
    #     1,
    #     p.POSITION_CONTROL,
    #     0,
    #     # p1[i][1],
    #     target_v,
    #     max_force
    # )
    f = -10.0*(p.getJointState(boxId,3)[0]-(0.2))
    # - p.getJointState(boxId,3)[3]
    # p.setJointMotorControl2(
    #     boxId,
    #     3,
    #     p.TORQUE_CONTROL,
    #     force = f
    #     # 500.0
    # )
    # p.setJointMotorControl2(
    #     boxId,
    #     3,
    #     p.VELOCITY_CONTROL,
    #     # force = 100)
    #     # force = -50*(p.getJointState(boxId,3)[0]%(2*np.pi)-(0.5)))
    #     force=f)
    p.setJointMotorControl2(
        boxId,
        3,
        p.POSITION_CONTROL,
        targetPosition = 0.2, targetVelocity = 0,
        positionGain = 0.1, velocityGain = 0.99, force= 5)
        # force = 100)
        # force = -50*(p.getJointState(boxId,3)[0]%(2*np.pi)-(0.5)))
        # force=f)
    print('angular displacement and force',p.getJointState(boxId,3)[0], p.getJointState(boxId,3), f)
    time.sleep(0.002)
    x_plot.append(x)
    y_plot.append(y)
    angle.append(p.getJointState(boxId,3)[0])
    f_plot.append(f)

#     force

# while p.isConnected():
#     p.stepSimulation()
#     p.setGravity(0, 0, -10)
#     p.setJointMotorControl2(
#         boxId,
#         0,
#         p.VELOCITY_CONTROL,
#         force = 0)
#     p.setJointMotorControl2(
#         boxId,
#         2,
#         p.VELOCITY_CONTROL,
#         force = 100)
#
#     p.setJointMotorControl2(
#         boxId,
#         3,
#         p.VELOCITY_CONTROL,
#         force=-50*(p.getJointState(boxId,3)[0]%(2*np.pi)-(0.5))
#     )
#     print('force', -50*(p.getJointState(boxId,3)[0]%(2*np.pi)-(0.5)) )
#     print('angular displacement and force',p.getJointState(boxId,3)[0], p.getJointState(boxId,3)[0]%(2*np.pi), p.getJointState(boxId,3)[3])

# time.sleep(0.5)
# print('xy from simulation', x_plot,
#       '\ny', y_plot)
print("simulation result", p.getJointState(boxId, 0), p.getJointState(boxId, 2))
print("joint dynamics", p.getDynamicsInfo(boxId,3))
g = 10
torque_grav = 2.5*g*np.sin(p.getJointState(boxId,3)[0])*1.0/2 + 0.2*g*np.sin(p.getJointState(boxId,3)[0])*1.0
print('torque_grav', torque_grav)
ax = plt.subplot(2, 1, 1)
# ax.set_aspect(1)
# ax.set_xlim(0, 10)
# ax.set_ylim(0, 10)
# plt.plot(
#     [5, 5,
#      10,
#      10, 5],
#     [0, 5,
#      5,
#      0, 0], color='r')  # draw the obstacle area
# ax.plot(x_plot, y_plot, color='purple')
# damp = []
# for i in range(1000):
#     exp = 0.7 * np.exp(-0.05*i/(2*2.7))  ### this probably should be the angular moment of inertia, rather than mass
#     damp.append(exp)
# ax.plot(damp)
ax.plot(angle)

ax = plt.subplot(2, 1, 2)

ax.plot(f_plot)
plt.show()
p.resetSimulation()