import tf
import math

p1 = [1,1,1]
o1 = [1.5,0,1]

p2 = [0,0,0]
o2 = [0,0,0]

x = 0.1* math.cos(1)
y = 0.1* math.sin(1)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(figsize=(15,15))
ax = fig.add_subplot(111, projection='3d')


ax.plot([1.,0 , 1-x], [1.,0, 1-y], [1.,0,1], markerfacecolor='k', markeredgecolor='k', marker='o', markersize=10, alpha=0.6)

# Putting the limits in the axes
plt.show()