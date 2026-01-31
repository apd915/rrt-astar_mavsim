import numpy as np

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
