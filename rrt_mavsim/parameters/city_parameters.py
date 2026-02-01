import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import map_2D_to_3D_planeMsg

#sets the number of dimensions for this algorithm
numDimensions = 2

# create random city map
city_width      = 2000.  # the city is of size (width)x(width)
building_height = 300.   # maximum height of buildings
num_blocks      = 4    # number of blocks in city
streetWidthRatio = 0.8   # percent of block that is street.
obstacleWidthRatio = 1.0 - streetWidthRatio
obstacleWidth_sigma = 20.0
minObstacleWidth = 20.0


#creates the operational altitude for the 2D planar planning algorithm
altitude = 100.0

#creates the start and end positions in 2D
startPosition_2D = np.array([[0.0],[0.0]])
endPosition_2D = np.array([[city_width],[city_width]])

mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[-altitude]])
n_hat = np.array([[0.0],[0.0],[1.0]])


plane_msg = MsgPlane(n_hat=n_hat,origin_3D=mapOrigin_3D)

startPosition_3D = map_2D_to_3D_planeMsg(vec_2D=startPosition_2D,
                                         plane_msg=plane_msg)
endPosition_3D = map_2D_to_3D_planeMsg(vec_2D=endPosition_2D,
                                       plane_msg=plane_msg)



#creates the x, y, and z limits for plotting 
x_limits = (-200, 2300)
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
