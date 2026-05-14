import numpy as np

# ==========================================
# THE LIDAR SANDBOX CONSTRAINTS
# Shrinking the map from 3km to a 100m perception bubble
# ==========================================

# 1. Obstacle Density
# 4 blocks per side means 16 total obstacles in the sandbox. 
numBlocks_perSide = 4

# 2. Obstacle Size
# Shrunk from 200m skyscrapers down to 10m structures (trees/buildings)
blockWidth = 10.0

# 3. Map Dimensions (The Lidar Horizon)
northEnd = 100.0
eastEnd = 100.0

# 4. Altitude
# Drones don't fly 3km high. Let's aim for a goal 15 meters in the air.
# Note: Assuming NED (North, East, Down) coordinates where negative Z is 'up'
altitudeEnd = -15.0
downEnd = -altitudeEnd

# 5. Boundary Conditions
startPosition_3D = np.array([[0.0],[0.0],[0.0]])
endPosition_3D = np.array([[northEnd],[eastEnd],[downEnd]])

numDimensions = 3

# degree of B-Spline
degree = 3

R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

# ==========================================
# PLOTTING CONSTRAINTS
# ==========================================
# Shrunk the padding from 200m down to 10m so the plot isn't zoomed way out
padding = 10.0

x_limits = (-padding, northEnd + padding)
y_limits = (-padding, eastEnd + padding)

# Sort Z-limits to prevent Matplotlib axis inversion errors (since altitude is negative)
z_min = min(-padding, altitudeEnd - padding)
z_max = max(-padding, altitudeEnd - padding)
z_limits = (z_min, z_max)

x_range = x_limits[1] - x_limits[0]
y_range = y_limits[1] - y_limits[0]
z_range = z_limits[1] - z_limits[0]

# gets the max
max_range = max(x_range, y_range, z_range)

# creates the list for the aspect ratio
aspect_ratio = [x_range/max_range, y_range/max_range, z_range/max_range]