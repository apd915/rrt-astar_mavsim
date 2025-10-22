import numpy as np
import parameters.aerosonde_parameters as MAV

# size of the waypoint array used for the path planner.  This is the
# maximum number of waypoints that might be transmitted to the path
# manager.
size_waypoint_array = 100

# airspeed commanded by planner
Va0 = MAV.u0

# max possible roll angle
phi_max = np.radians(25.)

# minimum turn radius
R_min = Va0**2. / MAV.gravity / np.tan(phi_max)
#print(R_min)

# create random city map
city_width      = 2000.  # the city is of size (width)x(width)
building_height_max = 300.0   # maximum height of buildings
building_height_min = 20.0
num_blocks      = 4    # number of blocks in city
street_width    = .8   # percent of block that is street.
obstacleWidthRatio = 0.3
obstacleWidth_sigma = 20.0
minObstacleWidth = 20.0
maxObstacleWidthRatio = 0.9
minObstacleWidthRatio = 0.1

#sets the scale
scale = 2500


#creates the operational altitude for the 2D planar planning algorithm
altitude = 100.0


#defines the list of edges between vertices here
edges_verticesIndices_2D = [[0,1],
                            [1,2],
                            [2,3],
                            [3,0]]


#defines the new list of edge vertices indices
edges_verticesIndices_3D = [[0,1], #pair 0 
                            [0,3], #pair 1 
                            [0,4], #pair 2
                            [1,2], #pair 3
                            [1,5], #pair 4
                            [2,3], #pair 5
                            [2,6], #pair 6
                            [3,7], #pair 7
                            [4,5], #pair 8
                            [4,7], #pair 9
                            [5,6], #pair 10
                            [6,7]] #pair 11

#these pairs refer to the edges. I have defined a list of edges, and these
#are the necessary indices for that
edgeVectorPairsIndices_list_3D = [[1,0],
                                  [0,2],
                                  [3,4],
                                  [5,6],
                                  [2,1],
                                  [8,9]]