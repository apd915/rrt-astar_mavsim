import numpy as np
from message_types.msg_waypoints import MsgWaypoints_SFC
from message_types.msg_flight_corridors import MsgFlightCorridor
from message_types.msg_world_map import MsgWorldMap
import heapq
from tools.intersections import intersectionDetected


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


#defines the function to smooth the path
def flight_corridors_smooth_path(waypoints_not_smooth: MsgWaypoints_SFC,
                                 world_map: MsgWorldMap)->MsgWaypoints_SFC:
    
    #gets the positions in the not smooth list
    waypoint_node_positions = waypoints_not_smooth.getAllPositions()

    positionsPairs_list, indices_list = getPositionsPairsList(waypoints_not_smooth=waypoints_not_smooth)

    #gets the valid positions, indices, flight corridors, and costs lists
    validPositions_list, validIndices_list, validFlightCorridors_list, cost_list =\
        getValidPositionPairs(positionsPairs_list=positionsPairs_list,
                              indicesPairs_list=indices_list,
                              world_map=world_map)

    #gets the number of nodes
    num_nodes = len(waypoint_node_positions)
    #gets the end node index
    end_node_index = num_nodes - 1
    #creates the list of edges. This is a list of lists. The First index denotes the 
    #starting node, and each element in the sublist corresponds to a node that the start
    #node connects to 
    edges = [[] for _ in range(num_nodes)]
    #creates the dictionary for the flight corridor list
    flight_corridors_dict = {}
    #iterates over the feasible edges list and categorizes them with the corresponding cost
    for (startNode_index, stopNode_index), tempCost, corridor in zip(validIndices_list, cost_list, validFlightCorridors_list):
        
        flight_corridors_dict[(startNode_index, stopNode_index)] = corridor
        #appends to the start nose index list of the edges
        edges[startNode_index].append((stopNode_index, tempCost))

    #creates the dictionary of min distance (cost) total for a particular transition
    dist = {}
    #creates a corresponding parent class for the parent node of each particular transition
    #associated with the parent that has the minimum cost
    parent = {}
    #creates the priority queue as a list
    priority_queue = []
    #creates the start transition which is the nonexistent one
    start_transition = (-1, 0)
    #sets the start cost
    start_cost = 0.0
    #sets the start cost for the start transition
    dist[start_transition] = start_cost
    #pushes onto the queue the start distance, and the start transition
    heapq.heappush(priority_queue, (dist[start_transition], start_transition[0], start_transition[1]))
    #iterates while there are items left in the priority queue
    while priority_queue:
        #gets the cost, previous node index and current node index for the item in the priority queue
        cost, previousNode_index, currentNode_index = heapq.heappop(priority_queue)
        #puts together the state (the previous to current node tuple)
        #TODO add end case
        if currentNode_index == end_node_index:
            #initializes the path as the current node index
            path = [currentNode_index]
            #initializes the previuos and current temps
            previous_temp, current_temp = previousNode_index, currentNode_index
            #iterates while we are in parent
            while (previous_temp, current_temp) in parent:
                #appends the previous node
                path.append(previous_temp)
                #gets the new previous and current temp
                current_temp, previous_temp = previous_temp, parent[(previous_temp, current_temp)]
            
            #reverses the path
            path.reverse()
            outputFlightCorridor_list = []
            #gets the positions
            position_list = []
            for i in range(len(path)):
                #gets the position
                tempPositions = waypoint_node_positions[path[i]]
                position_list.append(tempPositions)
            positionArray = np.concatenate(position_list, axis=1)
            #now that we have the path, we create the new waypoints
            for i in range(len(path) - 1):
                currentNode = path[i]
                nextNode = path[i + 1]
                #gets the flight corridor
                tempFlightCorridor = flight_corridors_dict[(currentNode, nextNode)]
                #appends it
                outputFlightCorridor_list.append(tempFlightCorridor)
            #creates the flight Corridor list
            waypointsOutput = MsgWaypoints_SFC()
            waypointsOutput.flightCorridors = outputFlightCorridor_list
            waypointsOutput.positions=position_list
            #returns the path and the total cost
            return waypointsOutput
        #iterates over the all the feasible edges in the edges list for the current node index
        for nextNode_index, marginalCost in edges[currentNode_index]:
            
            #rounds the marginal cost to the nearest tenths place
            marginalCost = np.round(marginalCost, 1)
            #checks if the previous node index is -1, which indicates that we are
            #at the start, at which point there is no angle to consider. otherwise,
            #we actually calculate the two angles

                
            #if we pass the angle check, we do a check for the costs
            #gets the new cost, which is the cost up to the current node in question
            #plus the marginal cost
            new_cost = cost + marginalCost
            #creates the transition state, which is the current node to the next node
            nextState = (currentNode_index, nextNode_index)
            #this part checks on any possible existing nodes for the transition state from current to next
            #in the distances dictionary. If there are any, then we check the previously calculated cost
            #if this new cost is more efficient, we discard the old one, and change the parent to correspond
            #to this new cost and next state
            if new_cost < dist.get(nextState, np.inf):
                
                #sets the distance at the new state as the new cost for that state
                dist[nextState] = new_cost
                #changes the parent to reflect this new previous node index
                parent[nextState] = previousNode_index
                #pushes this newly generatedone onto the heap
                heapq.heappush(priority_queue, (new_cost, currentNode_index, nextNode_index))


#defines helper function to get positions pairs list
def getPositionsPairsList(waypoints_not_smooth: MsgWaypoints_SFC):

    #gets all the positions in the corridor list
    corridorPositions_list = waypoints_not_smooth.getAllPositions()
    #gets  the length of the corridor positions list
    corridorPositions_len = len(corridorPositions_list)

    #creates the list of pairs of positions
    positionsPairs_list = []

    indices_list = []

    #section to get all the combinations of the positions list
    for i in range(corridorPositions_len - 1):

        #gets the start position
        start_position = corridorPositions_list[i]

        #then iterates over the positions after i
        for j in range((i + 1), corridorPositions_len):

            end_position = corridorPositions_list[j]

            #creates the tuple and saves it
            tempList = [start_position, end_position]

            positionsPairs_list.append(tempList)

            indices_list.append([i,j])

    return positionsPairs_list, indices_list



def getValidPositionPairs(positionsPairs_list: list[list[np.ndarray]],
                          indicesPairs_list: list[list[int]],
                          world_map: MsgWorldMap):
    
    validPositions_list = []
    validIndices_list = []
    flightCorridor_list = []
    cost_list = []

    for positionPair, indicesPair in zip(positionsPairs_list, indicesPairs_list):

        startPosition = positionPair[0]
        endPosition = positionPair[1]

        startIndex = indicesPair[0]
        endIndex = indicesPair[1]

        tempCorridor = MsgFlightCorridor(primaryPosition=startPosition,
                                            secondaryPosition=endPosition)
        
        tempCorridor_cost = tempCorridor.getCenterLength()

        
        tempCorridor_intersectionDetected = intersectionDetected(corridor=tempCorridor,
                                                                 world_map=world_map)
        
        if not tempCorridor_intersectionDetected:

            validPositions_list.append(positionPair)
            validIndices_list.append(indicesPair)
            flightCorridor_list.append(tempCorridor)
            cost_list.append(tempCorridor_cost)

    return validPositions_list, validIndices_list, flightCorridor_list, cost_list
