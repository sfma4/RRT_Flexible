import pybullet as p
from time import sleep
import time
import pybullet_data
import math
import matplotlib.pyplot as plt
import numpy as np

path = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\"
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
boxId = p.loadURDF(path+"TwoLink_5.urdf", basePosition=[0, 0, 0],
                   baseOrientation=p.getQuaternionFromEuler([math.pi/2, 0, 0]), useFixedBase=False)

p.setRealTimeSimulation(1)
# objId = p.loadURDF(path+"cube.urdf", basePosition=[7.5, 0, 2.5],
#                    useFixedBase=True, globalScaling=5)
target_v = 0.5  # motor angular speed rad/s
# target_v = [target_v, target_v]
max_force = 50  # max force exerted by motor
x_plot, y_plot = [], []
pos1 = np.linspace([2, 2, 0], [2, 2, 5], 100)
# pos2 = np.linspace([2, 5, 5],[5, 5, 5], 100)
# pos3 = np.linspace([5, 5, 5],[5, 2, 5], 100)
# pos = np.concatenate((pos1, pos2))
# path = np.concatenate((pos, pos3))
path = pos1
print('path', path)
print(path[:, 0])
print(path[:, 1])
# for i in range(len(path)+10):
#     x = p.getJointState(boxId, 0)[0]
#     y = p.getJointState(boxId, 2)[0]
#     x_plot.append(x)
#     y_plot.append(y)
#     if i >= len(path):
#         time.sleep(0.5)
#         print('sleep')
#     else:
#         p.stepSimulation()
#         # p.setGravity(0, 0, -10)
#
#         p.setJointMotorControl2(
#             boxId,
#             0,
#             p.POSITION_CONTROL,
#             path[:, 0][i],
#             target_v,
#             max_force
#         )
#         p.setJointMotorControl2(
#             boxId,
#             2,
#             p.POSITION_CONTROL,
#             path[:, 1][i],
#             target_v,
#             max_force
#         )
#         p.setJointMotorControl2(
#             boxId,
#             3,
#             p.VELOCITY_CONTROL,
#             force=0
#         )
#         p.setJointMotorControl2(
#             boxId,
#             4,
#             p.VELOCITY_CONTROL,
#             force=0
#         )
#         # p.setJointMotorControl2(
#         #     boxId,
#         #     4,
#         #     p.VELOCITY_CONTROL,
#         #     force=0
#         # )
#         time.sleep(0.00001)
#         x_plot.append(x)
#         y_plot.append(y)

while True:
    p.stepSimulation()
    # p.setGravity(0, 0, -10)

    # p.setJointMotorControl2(
    #     boxId,
    #     0,
    #     p.POSITION_CONTROL,
    #     path[:, 0][i],
    #     target_v,
    #     max_force
    # )
    # p.setJointMotorControl2(
    #     boxId,
    #     2,
    #     p.POSITION_CONTROL,
    #     path[:, 1][i],
    #     target_v,
    #     max_force
    # )
    p.setJointMotorControl2(
        boxId,
        3,
        p.VELOCITY_CONTROL,
        force=0
    )
    p.setJointMotorControl2(
        boxId,
        4,
        p.VELOCITY_CONTROL,
        force=0
    )
    # p.setJointMotorControl2(
    #     boxId,
    #     4,
    #     p.VELOCITY_CONTROL,
    #     force=0
    # )
    time.sleep(0.00001)
    # x_plot.append(x)
    # y_plot.append(y)
time.sleep(0.1)
print('pause to let gravity take effect')


# print('xy from simulation', x_plot, y_plot)
print("simulation result", p.getJointState(boxId, 0), p.getJointState(boxId, 2))
ax = plt.subplot(1, 1, 1)
ax.set_aspect(1)
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
plt.plot(
    [5, 5,
     10,
     10, 5],
    [0, 5,
     5,
     0, 0], color='r')  # 画处障碍物区域
ax.plot(x_plot, y_plot, color='purple')
plt.show()
p.resetSimulation()