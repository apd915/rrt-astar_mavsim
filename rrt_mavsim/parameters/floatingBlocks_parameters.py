import numpy as np

numBlocks_perSide = 4

blockWidth = 200.0

northEnd = 3000.0
eastEnd = 3000.0
altitudeEnd = -3000.0
downEnd = -altitudeEnd

startPosition_3D = np.array([[0.0],[0.0],[0.0]])
endPosition_3D = np.array([[northEnd],[eastEnd],[downEnd]])

numDimensions = 3


#degree of B-Spline
degree = 3

R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

#creates the x, y, and z limits for plotting 
x_limits = (-200, northEnd + 200)
y_limits = (-200, eastEnd + 200)
z_limits = (-200, altitudeEnd + 200)

x_range = x_limits[1] - x_limits[0]
y_range = y_limits[1] - y_limits[0]
z_range = z_limits[1] - z_limits[0]

#gets the max
max_range = max(x_range, y_range, z_range)
#creates the list  for the aspect ratio
aspect_ratio = [x_range/max_range, y_range/max_range, z_range/max_range]
