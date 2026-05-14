

import numpy as np
from scipy.optimize import linprog
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap

# --- TEMPORARY DEBUG COUNTERS ---
DEBUG_SKIPS = 0
DEBUG_LINPROG_CALLS = 0

def segment_aabb_intersection(p0, p1, box_min, box_max):
    """
    Lightning-fast Slab Method to check if a line segment intersects a 3D bounding box.
    Takes microseconds compared to linprog's milliseconds.
    """
    # Vector of the line segment
    d = p1 - p0
    
    # Prevent division by zero errors
    d_safe = np.copy(d)
    d_safe[d_safe == 0] = 1e-8
    
    # Calculate intersections with the 3 pairs of parallel bounding planes
    t1 = (box_min - p0) / d_safe
    t2 = (box_max - p0) / d_safe
    
    # Find the entry and exit parameters along the segment
    t_mins = np.minimum(t1, t2)
    t_maxs = np.maximum(t1, t2)
    
    # The segment enters the box at the MAX of the minimums
    t_enter = np.max(t_mins)
    # The segment exits the box at the MIN of the maximums
    t_exit = np.min(t_maxs)
    
    # If exit is greater than entry, AND it happens within the segment bounds (0 to 1), it's a hit!
    if t_exit >= t_enter and t_enter <= 1.0 and t_exit >= 0.0:
        return True
    return False

#defines the function to get the intersection with the sfc candidate and the world map
def intersectionDetected(corridor: MsgFlightCorridor,
                         world_map: MsgWorldMap):
    
    numDimensions = world_map.numDimensions_algorithm

    if numDimensions == 2:
        map_A_b_lists = world_map.get_Ab_2D()
    elif numDimensions == 3:
        map_A_b_lists = world_map.get_Ab_3D()

    A_corridor, b_corridor = corridor.getAbMatrices()
    
    # Flatten positions for easy vector dot-products
    corridor_start = corridor.primaryPosition.flatten()
    corridor_end = corridor.secondaryPosition.flatten()
    
    # We no longer use the massive length for the radius!
    # Just the physical width of the tube itself.
    corridor_radius = max(corridor.width, getattr(corridor, 'height', 0)) / 2.0
    
    # Define max obstacle radius (Diagonal of the block)
    max_obstacle_radius = 20.0 
    safe_cylinder_radius = corridor_radius + max_obstacle_radius

    # Pre-compute the line segment vector
    AB = corridor_end - corridor_start
    AB_squared = np.dot(AB, AB)
    if AB_squared == 0:
        AB_squared = 1e-6 # Prevent division by zero

    for AbMatrices in map_A_b_lists:
        tempObstacleA = AbMatrices[0]
        tempObstacle_b = AbMatrices[1]
        
        # Extract the center point and flatten it
        obstacle_center = AbMatrices[2].flatten()

        # ==========================================
        # STAGE 1: BROAD PHASE (Cylinder Distance)
        # ==========================================
        # Vector from start of line to obstacle
        AP = obstacle_center - corridor_start
        
        # Project AP onto AB to find the closest point on the infinite line
        t = np.dot(AP, AB) / AB_squared
        
        # Clamp 't' between 0 and 1 so we don't check past the ends of the segment!
        t = max(0.0, min(1.0, t))
        
        # Find the actual closest point on the segment
        closest_point = corridor_start + t * AB
        
        # Distance from the obstacle to that closest point
        distance_to_segment = np.linalg.norm(obstacle_center - closest_point)
        
        if distance_to_segment > safe_cylinder_radius:
            global DEBUG_SKIPS
            DEBUG_SKIPS += 1
            continue  

        # ==========================================
        # STAGE 2: NARROW PHASE (linprog)
        # ==========================================
        global DEBUG_LINPROG_CALLS
        DEBUG_LINPROG_CALLS += 1
        
        tempObstacleIntersectionOccurred = intersectionOccurred_Matrix(A1=A_corridor,
                                                                b1=b_corridor,
                                                                A2=tempObstacleA,
                                                                b2=tempObstacle_b)
        
        if tempObstacleIntersectionOccurred:
            return True
            
    return False




#Arguments:
#A1: the A matrix for object 1
#b1: the b vector for object 1
#A2: the A matrix for object 2
#b2: the b vector for object 2
def intersectionOccurred_Matrix(A1: np.ndarray,
                         b1: np.ndarray,
                         A2: np.ndarray,
                         b2: np.ndarray):
    
    #creates the new nex A and b matrices
    A_net = np.concatenate((A1, A2), axis=0)
    b_net = np.concatenate((b1, b2), axis=0)

    numDimensions = A_net.shape[1]

    #creates a dummy variable that the linprog function will use for the feasibility here
    dummyVariable = np.zeros(numDimensions)

    if numDimensions == 2:
        tempBounds = [(None, None), (None, None)]
    elif numDimensions == 3:
        tempBounds = [(None, None), (None, None), (None, None)]
    else: 
        tempBounds = [(None, None), (None, None)]


    #uses linprog to find whether or not there exists a viable solution to the problem here

    result = linprog(dummyVariable, A_ub=A_net, b_ub=b_net, bounds=tempBounds, method='highs')

    intersectionOccurredTemp = result.success

    return intersectionOccurredTemp