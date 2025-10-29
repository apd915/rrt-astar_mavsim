import numpy as np
#sets the parameters for the planar VTOL Map

#the valid region over which to generate the obstacles
mapLength = 10000.0
mapHeight = 2000.0

#the obstacle origin on the 2D plane ( in the new space created by this)
obstacleOrigin_2D = np.array([[0.0],[0.0]])


#sets the normal vector and point for the plane onto which the obstacles and the flight path are placed
#this is the origin in the 3D world frame of reference
obstacleOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

numObstacles = 50

#bounds upper and lower for the obstacle's size
obstacleMinWidth = 50.0
obstacleMaxWidth = 200.0

#because this will be a 3-D obstacle, we need to give it some depth
obstacleDepth = 50.0


