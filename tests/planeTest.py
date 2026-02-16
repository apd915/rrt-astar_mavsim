from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import *
import numpy as np
import rrt_mavsim.parameters.projection_parameters as PROJ
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D)
from matplotlib import cm


n_hat = np.array([[0.0],[-1.0],[0.0]])
n_hat = n_hat / np.linalg.norm(n_hat)

Q = getPlaneBasis(n_hat=n_hat)

#gets the basis vectors for plane
n_plane = Q[:,0:1]
d_plane = Q[:,1:2]

#creates plane origin
planeOrigin = np.array([[0.0],[-1.0],[-1.0]])

#creates the plane message
plane_msg = MsgPlane(n_hat=n_hat,
                     origin_3D=planeOrigin)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#plots stuff from the arrows
n_plane_ax = ax.quiver(planeOrigin[0,0],
                       planeOrigin[1,0],
                       planeOrigin[2,0],
                       n_plane[0,0],
                       n_plane[1,0],
                       n_plane[2,0],
                       normalize=True,
                       color='k')

d_plane_ax = ax.quiver(planeOrigin[0,0],
                       planeOrigin[1,0],
                       planeOrigin[2,0],
                       d_plane[0,0],
                       d_plane[1,0],
                       d_plane[2,0],
                       normalize=True,
                       color='k')


norm_plane_ax = ax.quiver(planeOrigin[0,0],
                       planeOrigin[1,0],
                       planeOrigin[2,0],
                       n_hat[0,0],
                       n_hat[1,0],
                       n_hat[2,0],
                       normalize=True,
                       color='green')

#creates the origin
origin = np.array([[0.0],[0.0],[0.0]])

# Plot the arrow for the 3D axes on the origin
n_3D = ax.quiver(origin[0,0], origin[1,0], origin[2,0], (PROJ.e1)[0,0], (PROJ.e1)[1,0], (PROJ.e1)[2,0], normalize=True, color='red')
e_3D = ax.quiver(origin[0,0], origin[1,0], origin[2,0], (PROJ.e2)[0,0], (PROJ.e2)[1,0], (PROJ.e2)[2,0], normalize=True, color='red')
d_3D = ax.quiver(origin[0,0], origin[1,0], origin[2,0], (PROJ.e3)[0,0], (PROJ.e3)[1,0], (PROJ.e3)[2,0], normalize=True, color='red')



#creates the limits for the plane in the plane frame
plane_x_limits = (-4.0, 4.0)
plane_y_limits = (-4.0, 4.0)

numSamples = 20

#creates the linspaces for x and y
x_plane_2D = np.linspace(plane_x_limits[0], plane_x_limits[1], numSamples)
y_plane_2D = np.linspace(plane_y_limits[0], plane_y_limits[1], numSamples)
X_2D, Y_2D = np.meshgrid(x_plane_2D, y_plane_2D)

X_3D = np.zeros((numSamples, numSamples))
Y_3D = np.zeros((numSamples, numSamples))
Z_3D = np.zeros((numSamples, numSamples))

for i in range(numSamples):
    for j in range(numSamples):

        X_temp_2D = X_2D[i,j]
        Y_temp_2D = Y_2D[i,j]

        pos_2D_temp = np.array([[X_temp_2D],[Y_temp_2D]])
        
        #gets the vector in 3D
        pos_3D_temp = map_2D_to_3D_planeMsg(vec_2D=pos_2D_temp,
                                            plane_msg=plane_msg)

        X_3D[i,j] = pos_3D_temp[0,0]
        Y_3D[i,j] = pos_3D_temp[1,0]
        Z_3D[i,j] = pos_3D_temp[2,0]


plane1 = ax.plot_surface(X_3D, Y_3D, Z_3D, shade=False)

#corrects the positioning so it doesn't do a false plot behind thing
n_plane_ax.set_sort_zpos(1000)
d_plane_ax.set_sort_zpos(1000)
norm_plane_ax.set_sort_zpos(1000)

n_3D.set_sort_zpos(1000)
e_3D.set_sort_zpos(1000)
d_3D.set_sort_zpos(1000)
plane1.set_sort_zpos(-1000)

testPoint = 0

# Set some limits so we can see it nicely
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(-5, 5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

testPoint = 0
