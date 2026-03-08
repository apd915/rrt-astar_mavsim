import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D

height = 2000.0
width = 6000.0
numTeeth = 6

barWidthRatio = 0.1
teethWidthRatio = 0.4
teethHeightRatio = 0.8

barHeight = 200.0

obstacleDepth = 300.0

numDimensions = 2

startPosition = np.array([[0.0],[0.0]])
endPosition = np.array([[height],[width]])

#creates the operational altitude for the 2D planar planning algorithm
altitude = 100.0

mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[-altitude]])
n_hat = np.array([[0.0],[0.0],[1.0]])

plane_msg = MsgPlane(n_hat=n_hat,origin_3D=mapOrigin_3D)


startPosition_3D = map_2D_to_3D(vec_2D=startPosition,
                                plane=plane_msg)
endPosition_3D = map_2D_to_3D(vec_2D=endPosition,
                              plane=plane_msg)


R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
degree = 3



#creates the x, y, and z limits for plotting 
x_limits = (-200, 6300)
y_limits = (-200, 2300)
z_limits = (-200, 500)

x_range = x_limits[1] - x_limits[0]
y_range = y_limits[1] - y_limits[0]
z_range = z_limits[1] - z_limits[0]

#gets the max
max_range = max(x_range, y_range, z_range)
#creates the list  for the aspect ratio
aspect_ratio = [x_range/max_range, y_range/max_range, z_range/max_range]


#sets the aspect ratio for the plotter

