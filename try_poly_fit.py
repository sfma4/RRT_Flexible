from poly_fit import ParabolicInterpolation
import numpy as np
pos = np.array([[0,0,0],[1,1,3],[2,3,6]])
time = np.array([[0],[1],[2]])
config = ParabolicInterpolation(name='Parabolic', q_via = pos, t_via=time)
p = []
for i in range(100):
    p = config.getPosition(i*2/100)
    # p = np.concatenate((p,p))
    print(p)
# print(config.getPosition(1))
