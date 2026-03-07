import numpy as np
import rrt_mavsim.parameters.planner_parameters as PLAN

# sets the part that extends before the start position
startExtension = 30.0
# sets the part that extends beyond the end position
endExtension = 30.0

# sets the initial segment length
segmentLength = 700.0

# sets the rho variable
rho = np.array([[1.0], [1.0], [1.0]])

# sets the B-Spline degree
degree = 3
# sets the number of intervals of interest
M = 10
# sets the minimum turning radius
MinTurnRadius = 300.0

# number of points per unit length
numPoints_perUnit = 5 * (degree + M) / segmentLength

# number of initial paths
numInitialPaths = 1

# the initial and final positions for the 2D and 3D cases.
initialPosition_2D = np.array([[0.0], [0.0], [-PLAN.altitude]])
finalPosition_2D = np.array([[PLAN.city_width], [PLAN.city_width], [-PLAN.altitude]])

initialPosition_3D = np.array([[0.0], [0.0], [0.0]])
finalPosition_3D = np.array([[PLAN.city_width], [PLAN.city_width], [-PLAN.city_width]])


# sets the width of the SFC
width = 150.0
# sets the height of the SFC (used only for 3D)
height = 150.0

# using the minimum turning radius and the width, we get the maximum angle between Safe flight Corridors
# for them to be allowed to be added to
gamma_min = np.arcsin((MinTurnRadius - width) / (MinTurnRadius))
Chi_max = np.pi - 2.0 * gamma_min
