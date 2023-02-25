import pybullet as p
from time import sleep
import time
import pybullet_data
import math
import matplotlib.pyplot as plt
import numpy as np
import time
timestr = time.strftime("%Y%m%d-%H%M%S")
from poly_fit import ParabolicInterpolation
import os
# sys.path.append("C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\data")
path = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\"
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


def simulation_try(p1):
    p.resetSimulation()
    p.setRealTimeSimulation(0)
    p.resetDebugVisualizerCamera(cameraDistance=12.5, cameraYaw=24.40, cameraPitch=-28,
                                 cameraTargetPosition=[2.3, 2.0, 0.85])

    # logid = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,
    #                     path + str(timestr) +  ".mp4")
    # print('logid', logid)
    # print('start position', p1[0][0], p1[0][1])
    boxId = p.loadURDF(path + "TwoLink_5.urdf", basePosition=[0, 0, 0],
                       baseOrientation=p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, 0]), useFixedBase=False)
    # boxId = p.loadURDF(path + "short_chain.urdf", basePosition=[0, 0, 0],
    #                    baseOrientation=p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, 0]), useFixedBase=False)

    objId = p.loadURDF(path + "cube.urdf", basePosition=[7.5, 0, 2.5],
                       useFixedBase=True, globalScaling=5)
    ob2 = p.loadURDF(path + "cube.urdf", basePosition=[8, 0, 5.5],
                       useFixedBase=True, globalScaling=1)
    # ob3 = p.loadURDF(path + "cube.urdf", basePosition=[7, 0, 6.5],
    #                    useFixedBase=True, globalScaling=1)
    # ob4 = p.loadURDF(path + "cube.urdf", basePosition=[6, 0, 6.5],
    #                  useFixedBase=True, globalScaling=1)
    redSlider = p.addUserDebugParameter("red", 0, 1, 1)
    greenSlider = p.addUserDebugParameter("green", 0, 1, 0)
    blueSlider = p.addUserDebugParameter("blue", 0, 1, 0)
    alphaSlider = p.addUserDebugParameter("alpha", 0, 1, 0.5)
    red = p.readUserDebugParameter(redSlider)
    green = p.readUserDebugParameter(greenSlider)
    blue = p.readUserDebugParameter(blueSlider)
    alpha = p.readUserDebugParameter(alphaSlider)
    p.changeVisualShape(ob2, -1, rgbaColor=[red, green, blue, alpha])
    # p.changeVisualShape(ob3, -1, rgbaColor=[red, green, blue, alpha])
    # p.setRealTimeSimulation(1)
    ### dynamics and property section, like friction etc
    # print('dynamics', p.getDynamicsInfo(boxId, 4))
    p.changeDynamics(boxId, 4, lateralFriction=0.2)
    # print('dynamics', p.getDynamicsInfo(boxId, 4))
    p.changeDynamics(boxId, 3, linearDamping=1.0, angularDamping=1.0)
    p.changeDynamics(boxId, 4, lateralFriction=0.25)
    p.changeDynamics(boxId, 6, lateralFriction=0.18)

    # print('underactuated joint dynamics', p.getJointInfo(boxId,3))
    target_v = 5.0  # motor angular speed rad/s and linear speed @@@@@@ how come this value change the position result?
    # target_v = [target_v, target_v]
    max_force = 800  # max force exerted by motor
    x_plot, y_plot = [], []
    theta1_plot, theta2_plot = [], []
    force = []
    joint1_pos = []
    p.resetJointState(boxId, 0, p1[0][0])
    p.resetJointState(boxId, 2, p1[0][1])
    # p.resetJointState(boxId, 3, p1[0][2])
    if len(p1[0]) > 3:
        p.resetJointState(boxId, 3, p1[0][2])
        p.resetJointState(boxId, 5, p1[0][3])
    else:
        p.resetJointState(boxId, 3, 0.0)
        p.resetJointState(boxId, 5, 0.0)
        # p.resetJointState(boxId, 5, 0.1)


    # p.setTimeStep(0.05)   ### seems like changing this will heavily affect the fidelity
    time.sleep(0.2)
    for i in range(len(p1)):
        # if i == 0:
        #     for k in range(1000):
        p.stepSimulation()
        p.setGravity(0, 0, -10)
        p.setJointMotorControl2(
            boxId,
            0,
            p.POSITION_CONTROL,
            p1[:, 0][i],
            target_v,
            max_force
        )
        p.setJointMotorControl2(
            boxId,
            2,
            p.POSITION_CONTROL,
            p1[:, 1][i],
            target_v,
            max_force
        )
        if len(p1[0]) > 3:
            p.setJointMotorControl2(
                boxId,
                3,
                p.POSITION_CONTROL,
                targetPosition=p1[0][2], targetVelocity=0,
                positionGain=0.1, velocityGain=0.9, force=20)
            p.setJointMotorControl2(
                boxId,
                5,
                p.POSITION_CONTROL,
                targetPosition=p1[0][3], targetVelocity=0,
                # positionGain=0.1, velocityGain=0.9, force=10)
                positionGain=0.4, velocityGain=0.5, force=20)
        else:
            p.setJointMotorControl2(
                boxId,
                3,
                p.POSITION_CONTROL,
                targetPosition=0.2, targetVelocity=0,
                positionGain=0.1, velocityGain=0.9, force=20)
            p.setJointMotorControl2(
                boxId,
                5,
                p.POSITION_CONTROL,
                targetPosition=0.1, targetVelocity=0,
                positionGain=0.4, velocityGain=0.5, force=20)

        # p.setJointMotorControl2(
        #     boxId,
        #     3,
        #     p.TORQUE_CONTROL,
        #     force=500 * (p.getJointState(boxId, 3)[0] % (2 * np.pi) - 0.5))
        # print('angular displacement and force', p.getJointState(boxId, 3)[0] % (2 * np.pi),
        #       500 * (p.getJointState(boxId, 3)[0] % (2 * np.pi) - 0.5))
        # p.setJointMotorControl2(
         # p.setJointMotorControl2(  ### this is to set the joint 5 to free rotating state
        #     boxId,
        #     4,
        #     p.VELOCITY_CONTROL,
        #     force=0
        # )

                # print("k", k)
        # for j in range(100):
        #     p.stepSimulation()
        #
        #     p.setGravity(0, 0, -10)
        #     p.setJointMotorControl2(  ### this is to set the joint to free rotating state
        #         boxId,
        #         3,
        #         p.VELOCITY_CONTROL,
        #         force=0
        #     )
        # p.stepSimulation()
        # p.setGravity(0, 0, -10)
        # p.enableJointForceTorqueSensor(boxId, 4)
        # p.setJointMotorControl2(
        #     boxId,
        #     0,
        #     p.POSITION_CONTROL,
        #     p1[:, 0][i] ,
        #     target_v,
        #     max_force
        # )
        # p.setJointMotorControl2(
        #     boxId,
        #     2,
        #     p.POSITION_CONTROL,
        #     p1[:, 1][i],
        #     target_v,
        #     max_force
        # )
        # p.setJointMotorControl2(    ### this is to set the joint to free rotating state
        #     boxId,
        #     3,
        #     p.VELOCITY_CONTROL,
        #     force=0
        # )
        time.sleep(0.0001)
        x = p.getJointState(boxId, 0)[0]
        y = p.getJointState(boxId, 2)[0]
        theta = p.getJointState(boxId, 3)[0]
        theta_2 = p.getJointState(boxId, 5)[0]
        f_1 = p.getJointState(boxId, 4)[2]
        # print('join force', f_1)
        f = p.getJointState(boxId, 4)[0]
        x_plot.append(x)
        y_plot.append(y)
        theta1_plot.append(theta)
        theta2_plot.append(theta_2)
        force.append(f)
        joint1_pos.append(f_1)
        # if abs(x - p1[-1][0]) < 0.1 and abs(y - p1[-1][1]) < 0.1:
        #     print('loop end')
        #     break
    # for i in range(500):
    #     p.stepSimulation()
    #
    #     p.setGravity(0, 0, -10)
    #     p.setJointMotorControl2(    ### this is to set the joint to free rotating state
    #         boxId,
    #         3,
    #         p.VELOCITY_CONTROL,
    #         force=0
    #     )
    #     time.sleep(0.00001)
    #     x = p.getJointState(boxId, 0)[0]
    #     y = p.getJointState(boxId, 2)[0]
    #     theta = p.getJointState(boxId, 3)[0]
    #     f_1 = p.getJointState(boxId, 4)[2]
    #     # print('join force', f_1)
    #     f = p.getJointState(boxId, 4)[0]
    #     x_plot.append(x)
    #     y_plot.append(y)
    #     force.append(f)
    #     joint1_pos.append(f_1)
    #     # a = "i"
    #     if  i == 300:
    #         print("freefloating")
    #     if  i == 500:
    #         print("freefloating")
    #     if  i == 800:
    #         print("freefloating")
    # # time.sleep(5.0)
    # print('pause to let gravity take effect')

        ### this section code make sure the recorded value gets updated for the bounce
    end_pos = [x_plot[-1] + 0.5*np.sin(theta) + 0.5*np.sin(theta_2), y_plot[-1] - 0.5* np.cos(theta) - 0.5*np.cos(theta_2)]
    end_pos = [p.getLinkState(boxId,6)[0][0], p.getLinkState(boxId,6)[0][2]]
    # p.stopStateLogging(logid)

    # print("theta angle at last", theta)
    # print('end position',end_pos, x_plot[-1], y_plot[-1])
    # print('xy from simulation', x_plot,
    #        y_plot)
    # print("simulation result", p.getJointState(boxId, 0), p.getJointState(boxId, 2), p.getJointState(boxId, 3))
    # print('join force', f_1)

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
    if len(p1[0]) < 3:
        ax.plot(x_plot, y_plot, color='purple', marker='v', markersize=2)
    else:
        pass
    # x_tip = x + np.sin(p.getJointState(boxId, 3)[0]) * 1.0
    # y_tip = y - np.cos(p.getJointState(boxId, 3)[0]) * 1.0
    # ax.plot([end_pos[0],end_pos[1]], [x,y],color='blue', linewidth='3',alpha=0.5)
    # ax.plot(end_pos[0], end_pos[1], color='black', marker='o', markersize=3,alpha=0.5)
    # plt.show()

    return end_pos, [x_plot,y_plot], [theta, theta_2]  ## thetas represent the final state of the underactuated joints
