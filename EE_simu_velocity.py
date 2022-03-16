import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from model import simulation_solve

t0 = 0
t1 = 0.5
dt = 0.001
x0 = 0
t = np.arange(t0,t1,dt)
n = len(t)
# speed = np.ones(n)*10
# speed[0] = 0
speed = [5] # set initial speed as 5 when contact starts
m = 1
c = 2
k = 30
a = [0]  #initial acceleration
x = [x0] # initial displacement
F = [5]
for i in range(1,n):
    print()
    a.append((c*speed[i-1]+k*x[i-1])/m)
    speed.append(speed[i-1]-a[i-1]*dt)
    x.append(x[i-1]+ speed[i-1]*dt)
    print(x[-1])
    # F.append(c*speed[i] + k*x[i])
# a.append(a[-1])
print(max(a), 'max a')
plt.plot(t, a)
plt.plot(t, speed)
plt.plot(t, x)

plt.legend(["a", "speed", "x"])
plt.show()

# with initial speed,