import numpy as np
import parameters.planner_parameters as PLAN

#sets the part that extends before the start position
startExtension = 30.0
#sets the part that extends beyond the end position
endExtension = 30.0

#sets the initial segment length
segmentLength = 400.0

#sets the B-Spline degree
degree = 3
#sets the number of intervals of interest
M = 10
#sets the minimum turning radius
MinTurnRadius = 300.0

#number of points per unit length
numPoints_perUnit = (degree + M)/segmentLength

#number of initial paths
numInitialPaths = 1

#sets the initial start and end positions
initialPosition = np.array([[0.0],[0.0],[0.0]])
finalPosition = np.array([[PLAN.city_width],[PLAN.city_width],[-PLAN.city_width]])

#sets the width of the SFC
width = 50.0
#sets the height of the SFC
height = 50.0