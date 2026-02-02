import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


vertices_list = [np.array([[0.0],[0.0],[0.0]]),
            np.array([[100.0],[0.0],[0.0]]),
            np.array([[100.0],[50.0],[0.0]]),
            np.array([[0.0],[50.0],[0.0]])]

vertices_list.append(vertices_list[0])

vertices = np.concatenate((vertices_list), axis=1)

ax.plot(vertices[0,:],vertices[1,:],vertices[2,:], color='red', linewidth=2)

plt.show()

testPoint = 0
