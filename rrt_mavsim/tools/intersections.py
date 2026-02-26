

import numpy as np
from scipy.optimize import linprog
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap



#defines the function to get the intersection with the sfc candidate and the world map
def intersectionDetected(corridor: MsgFlightCorridor,
                         world_map: MsgWorldMap):
    

    #gets the number of dimensions of this problem
    numDimensions = world_map.numDimensions_algorithm

    #if this is a 2D problem
    if numDimensions == 2:
        map_A_b_lists = world_map.get_Ab_2D()
    elif numDimensions == 3:
        map_A_b_lists = world_map.get_Ab_3D()



    #gets the corridor A and b matrices
    A_corridor, b_corridor = corridor.getAbMatrices()
    
    intersection = False

    #iterates over all of the A b matrices
    for AbMatrices in map_A_b_lists:

        tempObstacleA = AbMatrices[0]
        tempObstacle_b = AbMatrices[1]

        #with the obstacle A and b, we check for intersection
        tempObstacleIntersectionOccurred = intersectionOccurred_Matrix(A1=A_corridor,
                                                                b1=b_corridor,
                                                                A2=tempObstacleA,
                                                                b2=tempObstacle_b)
        
        #if we find an intersection, then we return true
        if tempObstacleIntersectionOccurred:
            return True
        


    #if we don't find an intersection, we return false
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

    #creates a dummy variable that the linprog function will use for the feasibility here
    dummyVariable = np.zeros(A_net.shape[1])

    #uses linprog to find whether or not there exists a viable solution to the problem here

    result = linprog(dummyVariable, A_ub=A_net, b_ub=b_net, bounds=[(None, None), (None, None)], method='highs')

    intersectionOccurredTemp = result.success

    return intersectionOccurredTemp
