import numpy as np
from numpy import exp
from matplotlib.pyplot import *
import matplotlib.pyplot as plt
from matplotlib import pyplot as mp


def gaussian(v, x, mu, sig):
    return v*(exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.))))

def generate_velocities(v, trajectory):
    """function that generates the velocity profile  from the maximum velocity"""
    position = len(trajectory)/2
    spread = 68
    velocities = [v*exp(-((i-position)**2.)/(2.*(spread**2.))) for i in range(len(trajectory))]

    #velocities = gaussian(v, np.linspace(0, 100, len(trajectory)), position, spread)

    #for mu, sig in [(-1, 1), (0, 2), (2, 3)]:
     #   velocities = gaussian(np.linspace(-3, 3, 120), mu, sig)
      #  mp.plot(gaussian(np.linspace(-3, 3, 120), mu, sig))

    #mp.show()

    return velocities

trajectory = [i for i in range(100)]
#print(trajectory)
v = generate_velocities(5, trajectory)
#print(v)
print(len(trajectory), len(v))

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(trajectory, v)
plt.show()

