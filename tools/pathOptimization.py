import numpy as np
from message_types.msg_waypoints import MsgWaypoints_SFC
from message_types.msg_flight_corridors import MsgFlightCorridor
from message_types.msg_world_map import MsgWorldMap
import heapq


#contains functions that help optimize the SFC Path, once a valid path has been found in the tree expansion
#this is by finding the minimum original path from the main paths avaliable,
#and then the smooth path function which skips points and generates a best path with dijkstra's algorithm



#gets the minimum path off of a tree
def findMinimumPath(tree: MsgWaypoints_SFC,
                    endPosition: np.ndarray):
    
    #gets the list of end connecting node indices and positions
    end_connecting_node_indices = []
    end_connecting_node_positions = []

    #iterates over all the paths in the tree
    for pathIndex in range(tree.numPositions):
        #when a connection to the goal is found, we add this to the list of possible paths
        if tree.connectsToGoal[pathIndex] == True:

            endConnectingPosition = tree.positions[pathIndex]

            end_connecting_node_indices.append(pathIndex)
            end_connecting_node_positions.append(endConnectingPosition)

    #gets the tree cost array
    treeCostArray = np.array(tree.costs)
    minimumCostIndex = np.argmin(treeCostArray[end_connecting_node_indices])

    #creates the path variable
    path = [end_connecting_node_indices[minimumCostIndex]]

    #gets the parent node initialized
    parent_node = tree.parents[end_connecting_node_indices[minimumCostIndex]]

    while parent_node >= 1:
        path.insert(0, int(parent_node))
        parent_node = tree.parents[int(parent_node)]

    #inserts the parent node
    path.insert(0,0)



    #now with this waypoint list, we get the min path waypoints
    minPathWaypoints = MsgWaypoints_SFC()

    for i, newPathIndex in enumerate(path):
        
        #gets the temporary position
        tempPosition = tree.getPosition(index=newPathIndex)

        #gets the temporary cost
        tempCost = tree.getCost(index=newPathIndex)

        #adds this to the waypoints
        minPathWaypoints.add(position=tempPosition,
                             cost=tempCost,
                             parent=np.inf,
                             connectsToGoal=False)
    #adds the end position
    minPathWaypoints.add(position=endPosition,
                         cost=np.inf,
                         parent=np.inf,
                         connectsToGoal=False)

    numWaypoints = minPathWaypoints.numPositions
    numCorridors = numWaypoints - 1

    for i in range(numCorridors):

        startTempPosition = minPathWaypoints.getPosition(index=i)
        endTempPosition = minPathWaypoints.getPosition(index=(i+1))

        #gets the flight corridor
        tempCorridor = MsgFlightCorridor(primaryPosition=startTempPosition,
                                            secondaryPosition=endTempPosition)

        minPathWaypoints.addSFC(sfc=tempCorridor)

    return minPathWaypoints


def flight_corridors_smooth_path(waypoints_not_smooth: MsgWaypoints_SFC,
                                 world_map: MsgW)


