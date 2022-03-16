import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from model import simulation_solve

t0 = 0
t1 = 60
step_time = 0.1

x_init = [10, 2]
t = np.arange(t0, t1+1, step_time)

property = [4,  # damping
            2,  # spring
            20, # mass
            5]  # force input
def simulation_solve(x, t):
    # this function solve the resulted position x in a function of time, t
    # at this stage, to keep it simple, only one direction is considered
    property = [4,  # damping
                2,  # spring
                20,  # mass
                0]  # force input
    c = property[0]
    k = property[1]
    m = property[2]
    F = property[3]
    # x_init = init_condition[0][0]

    dx1dt = x[1]
    dx2dt = (F - c*x[1] - k*x[0] ) / m
    dxdt = [dx1dt, dx2dt]
    return dxdt

x = odeint(simulation_solve, x_init,t)
x1 = x[:, 0]
x2 = x[:, 1]

plt.plot(t, x1)
plt.plot(t, x2)
plt.legend(["x1", "x2"])
plt.show()