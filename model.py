import numpy as np
from scipy.integrate import odeint
def simulation_solve(x, t):
    # this function solve the resulted position x in a function of time, t
    # at this stage, to keep it simple, only one direction is considered
    property = [4,  # damping
                2,  # spring
                20,  # mass
                10]  # force input
    c = property[0]
    k = property[1]
    m = property[2]
    F = property[3]
    # x_init = init_condition[0][0]

    dx1dt = x[1]
    dxdt = [dx1dt, dx2dt]
    return dxdt