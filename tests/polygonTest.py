#implements a test to plot things using matplotlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


#creates the points to plot
v0 = np.array([0.0,0.0,0.0])
v1 = np.array([1.0,0.0,0.0])
v2 = np.array([1.0,1.0,0.0])
v3 = np.array([0.0,1.0,0.0])
v4 = np.array([0.0,0.0,1.0])
v5 = np.array([1.0,0.0,1.0])
v6 = np.array([1.0,1.0,1.0])
v7 = np.array([0.0,1.0,1.0])


faces = [[v0, v1, v2, v3],
         [v0, v1, v5, v4],
         [v0, v3, v7, v4],
         [v2, v3, v7, v6],
         [v1, v2, v6, v5],
         [v4, v5, v6, v7]]

# Create the mesh
cube = Poly3DCollection(faces, alpha=1.0, facecolor='red', edgecolor='k')

ax.add_collection3d(cube)

# Set equal aspect ratio
ax.set_box_aspect([1, 1, 1])

# Set limits
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

plt.show()
