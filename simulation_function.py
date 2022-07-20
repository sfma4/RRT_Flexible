import pybullet as p
from time import sleep
import time
import pybullet_data
import math
from poly_fit import ParabolicInterpolation
import os
# sys.path.append("C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\data")
path = "C:\\Users\\Shang-G14\\EE_simulation\\bullet\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\"
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

boxId = p.loadURDF(path+"TwoLink_2.urdf", basePosition=[2, 0, 2],
                   baseOrientation=p.getQuaternionFromEuler([math.pi/2, 0, 0]), useFixedBase=False)


objId = p.loadURDF(path+"cube.urdf", basePosition=[7.5, 0, 2.5],
                   useFixedBase=True, globalScaling=5)

def simulation_try(p1):
    target_v = 10  # motor angular speed rad/s
    # target_v = [target_v, target_v]
    max_force = 100  # max force exerted by motor
    for i in range(len(p1)):
        p.stepSimulation()
        p.setGravity(0, 0, -10)
        p.setJointMotorControl2(
            boxId,
            0,
            p.POSITION_CONTROL,
            p1[:, 1][i],
            target_v,
            max_force
        )
        p.setJointMotorControl2(
            boxId,
            2,
            p.POSITION_CONTROL,
            p1[:, 0][i],
            target_v,
            max_force
        )
        p.setJointMotorControl2(
            boxId,
            4,
            p.VELOCITY_CONTROL,
            force=0
        )
        time.sleep(0.0001)
    print(p.getJointState(boxId, 2), p.getJointState(boxId, 0))
    p.resetSimulation()
# def main():
#
#
# if __name__ == '__main__':
#     main()
