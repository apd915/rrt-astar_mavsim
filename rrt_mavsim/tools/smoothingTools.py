from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.tools.intersections import intersectionDetected
import heapq
import numpy as np




#this function smooths the paths, and does it a bit different than the original
#algorithm. This checks for more combinations along the way.
def flight_corridors_smooth_path_constrainted(waypoints_not_smooth: MsgWaypoints_SFC,
                                     world_map: MsgWorldMap,
                                     angle_max: float)->MsgWaypoints_SFC:
    #gets the waypoints positions
    waypoint_node_positions = waypoints_not_smooth.getAllPositions()
    #gets the positions pairs list and the indices list
    positionsPairs_list, indicesPairs_list = getPositionsPairsList(waypoints_not_smooth=waypoints_not_smooth)
    #gets the valid positions pairs list
    validPositionPairs_list, validIndicesPairs_list, validFlightCorridor_list, costs_list =\
          getValidPositionsPairs(positionsPairs_list=positionsPairs_list,
                                 indicesPairs_list=indicesPairs_list,
                                 world_map=world_map)
    #gets the number of nodes
    num_nodes = len(waypoint_node_positions)
    #gets the end node index
    end_node_index = num_nodes - 1
    #creates the list of edges. This is a list of lists. The First index denotes the 
    #starting node, and each element in the sublist corresponds to a node that the start
    #node connects to 
    edges = [[] for _ in range(num_nodes - 1)]
    #creates the dictionary for the flight corridor list
    flight_corridors_dict = {}
    #iterates over the feasible edges list and categorizes them with the corresponding cost
    for (startNode_index, stopNode_index), tempCost, corridor\
          in zip(validIndicesPairs_list, costs_list, validFlightCorridor_list):
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
            #TODO -- Note to myself for Nov 8. Start here when starting again.
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
            waypointsOutput = MsgWaypoints_SFC(numDimensions=world_map.numDimensions_algorithm)
            waypointsOutput.flightCorridors = outputFlightCorridor_list
            waypointsOutput.positions = position_list
            #returns the path and the total cost
            return waypointsOutput
        #iterates over the all the feasible edges in the edges list for the current node index
        for nextNode_index, marginalCost in edges[currentNode_index]:
            
            #rounds the marginal cost to the nearest tenths place
            marginalCost = np.round(marginalCost, 1)
            #checks if the previous node index is -1, which indicates that we are
            #at the start, at which point there is no angle to consider. otherwise,
            #we actually calculate the two angles
            if previousNode_index != -1:

                #gets the primary flight corridor (previous node to current node)
                primaryFlightCorridor = flight_corridors_dict[(previousNode_index, currentNode_index)]
                secondaryFlightCorridor = flight_corridors_dict[(currentNode_index, nextNode_index)]
                #gets the angle between the primary and secondary flight corridors
                angle_primaryToSecondary_mag = getSFCAngle_magnitude(sfc_1=primaryFlightCorridor,
                                                                 sfc_2=secondaryFlightCorridor)

                #if the angle magnitude is greater than the maximum allowable angle magntiude, we don't consider this one
                if angle_primaryToSecondary_mag >= angle_max:
                    continue
                
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
            tempCost = dist.get(nextState, np.inf)
            
            if new_cost < tempCost:
                
                #sets the distance at the new state as the new cost for that state
                dist[nextState] = new_cost
                #changes the parent to reflect this new previous node index
                parent[nextState] = previousNode_index
                #pushes this newly generatedone onto the heap
                heapq.heappush(priority_queue, (new_cost, currentNode_index, nextNode_index))
            potato = 0
    potato = 0

#smooths the path, but without taking into account the angle between flight corridors
def flight_corridors_smooth_path_unconstrainted(waypoints_not_smooth: MsgWaypoints_SFC,
                                                world_map: MsgWorldMap):


    positionPairsList, indicesPairsList = getPositionsPairsList(waypoints_not_smooth=waypoints_not_smooth)


    validPositionsPairs_list, validIndicesPairs_list, flightCorridor_list, cost_list = getValidPositionsPairs(positionsPairs_list=positionPairsList,
                                                                                                    indicesPairs_list=indicesPairsList,
                                                                                                    world_map=world_map)
    #gets the grouped indices
    groupedIndices = organizeIndicesPairs(indicesPairsList=validIndicesPairs_list)
    #gets the total number of nodes
    numNodes = waypoints_not_smooth.getNumNodes()
    #creates the set of indices as the unvisited list
    unvisited = set(range(0,numNodes))

    #creates the list of all nodes and their parents and costs
    allNodes = list(unvisited)
    allCosts = [np.inf] * numNodes
    allParents = [np.inf] * numNodes

    #gets the ending node
    endingNode = allNodes[-1]

    #sets the node zero as the parent
    allCosts[0] = 0.0

    #goes through while there are unvisited nodes
    while unvisited:

        #gets the minimum cost of the unvisited nodes
        minCost = np.inf
        minCostNode = 0

        for node in unvisited:
            #gets the cost
            tempCost = allCosts[node]
            if tempCost < minCost:
                minCost = tempCost
                minCostNode = node

        #because of the way I have this structured, there are no nodes connecting to the last way
        #so, we skip the step, and say that it's been visited already
        if minCostNode != endingNode:
        
            #gets the connecting nodes for the min cost unvisited node
            connectingNodes_list = groupedIndices[minCostNode]

            #iterates all of the connecting nodes
            for connectingNode in connectingNodes_list:

                #gets the pair
                tempPair = [minCostNode, connectingNode]

                pairIndex = validIndicesPairs_list.index(tempPair)

                #gets the connectionCost to the next node
                connectionCost = cost_list[pairIndex]

                #gets the cost of the current node
                currentNodeCost = allCosts[minCostNode]

                #gets the next node cose as sum of the two above
                nextNodeCost = currentNodeCost + connectionCost

                #checks if this cost is less than the current cost for that node in the table
                if nextNodeCost < allCosts[connectingNode]:
                    #if this is the case, we set the current node as that next node's parent
                    #and save this cost
                    allCosts[connectingNode] = nextNodeCost
                    allParents[connectingNode] = minCostNode


                
                testPoint = 0

        #removes the current node from the unvisited list
        unvisited.discard(minCostNode)


    #gets the path list
    minCostList = getMinPath(allParentsList=allParents)


    #with the min cost list, we need to get the corresponding safe flight corridors,
    #and create smooted waypoints
    for i in range(len(minCostList) - 1):
        currentNode_index = minCostList[i]
        nextNode_index = minCostList[i+1]

        #gets the current safe flight corridor from the above list
        listNodes = [currentNode_index, nextNode_index]
        #gets the list index
        list



    #gets the 
    testPoint = 0

#defines the function to find the angle between two safe flight corridors
def getSFCAngle_magnitude(sfc_1: MsgFlightCorridor,
                          sfc_2: MsgFlightCorridor):
    
    #gets the Forward pointing normal vector of the sfc_1
    sfc_1_normal = sfc_1.primaryToSecondary_unit
    #same for the sfc 2
    sfc_2_normal = sfc_2.primaryToSecondary_unit

    #now with these two, we need to get the angle between them
    dotProduct = (sfc_1_normal.T @ sfc_2_normal)[0,0]

    #gets the magnitudes
    sfc_1_mag = np.linalg.norm(sfc_1_normal)
    sfc_2_mag = np.linalg.norm(sfc_2_normal)

    #gets the normalized dot
    dotProduct_normal = dotProduct / (sfc_1_mag*sfc_2_mag)

    #gets the angle between the vectors
    theta = np.arccos(dotProduct_normal)

    #return theta
    return theta

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

            potato = 0

    return positionsPairs_list, indices_list


#helper function to get the valid positions and indices list from all the pairs
def getValidPositionsPairs(positionsPairs_list: list[list[np.ndarray]],
                           indicesPairs_list: list[list[int]],
                           world_map: MsgWorldMap):

        validPositions_list = []
        validIndices_list = []
        #creates a corresponding flight corridor list
        flightCorridor_list = []

        #creates the corresponding cost list
        cost_list = []

        intersectionsDetectedList = []

        #iterates over each position pair in the list
        for positionPair, indicesPair in zip(positionsPairs_list, indicesPairs_list):

            #gets the start position
            startPosition = positionPair[0]
            endPosition = positionPair[1]

            #gets the start index and end index
            startIndex = indicesPair[0]
            endIndex = indicesPair[1]

            #creates the corridor            
            tempCorridor = MsgFlightCorridor(numDimensions=world_map.numDimensions_algorithm,
                                             primaryPosition=startPosition,
                                             secondaryPosition=endPosition,
                                             primaryPosition_index=startIndex,
                                             secondaryPosition_index=endIndex)

            #gets the length of the temp corridor, from the primary to the secondary position
            tempCorridor_cost = tempCorridor.primaryToSecondary_distance

            
            #call the intersection detected map
            tempCorridor_intersectionDetected = intersectionDetected(corridor=tempCorridor,
                                                                     world_map=world_map)

            intersectionsDetectedList.append(tempCorridor_intersectionDetected)

            if not tempCorridor_intersectionDetected:

                validPositions_list.append(positionPair)
                validIndices_list.append(indicesPair)
                flightCorridor_list.append(tempCorridor)

                #appends the cost
                cost_list.append(tempCorridor_cost)
        
        return validPositions_list, validIndices_list, flightCorridor_list, cost_list



def organizeIndicesPairs(indicesPairsList: list[list[int]]):

    indicesGroups = {}
    for a, b in indicesPairsList:

        indicesGroups.setdefault(a,[]).append(b)


    return indicesGroups


#helper function to go through and get the min path
def getMinPath(allParentsList: list):

    currentNodeIndex = int(len(allParentsList) - 1)
    finished = False

    minCostList = []
    minCostList.append(currentNodeIndex)

    while currentNodeIndex != 0:

        currentNodeIndex = allParentsList[currentNodeIndex]

        minCostList.append(currentNodeIndex)

    #reverses the minCostList
    minCostList.reverse()


    return minCostList

        

