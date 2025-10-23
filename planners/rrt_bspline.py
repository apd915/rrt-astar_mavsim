#the rrt function for generating B-Splines
import numpy as np
from planners.bspline_parameters import BsplineParameters
from message_types.msg_bspline_conditions import MsgBsplineConditions
from message_types.msg_world_map import MsgWorldMap
from message_types.msg_waypoints import MsgWaypoints_SFC
from message_types.msg_flight_corridors import MsgFlightCorridor
import parameters.planner_parameters as PLAN
import random
import time
import scipy as sp
from tools.intersections import intersectionOccurred
import heapq


#creates the RRT B-Spline class
class RRTBSpline:

    #creates the init function
    def __init__(self,
                 numDimensions: int,
                 M: int,
                 Va: float,
                 rho: np.ndarray,
                 step_length: float,
                 numDesiredInitPaths: int):
        
        #saves all of them
        self.numDimensions = numDimensions
        self.M = M
        self.Va = Va
        self.rho = rho
        self.stepLength = step_length
        self.numDesiredInitPaths = numDesiredInitPaths

        #creates the B-Splines
        self.bsplineParam = BsplineParameters()

    #creates the update function
    def generatePaths(self,
                      worldMap: MsgWorldMap,
                      startPosition: np.ndarray,
                      endPosition: np.ndarray,
                      segmentLength: float,
                      altitude: float):
        
        self.startPosition = startPosition
        self.endPosition = endPosition

        self.segmentLength = segmentLength

        self.tree = MsgWaypoints_SFC(numDimensions=self.numDimensions)

        (self.tree).add(position=self.startPosition,
                        parent=np.inf,
                        cost=np.float64(0.0),
                        connectsToGoal=False)
        
        #sets the number of paths found
        currentNumPaths_found = 0


        while currentNumPaths_found < self.numDesiredInitPaths:

            foundNewPathFlag = self.extend_initial_tree(worldMap=worldMap,
                                                        altitude=altitude)

            currentNumPaths_found += foundNewPathFlag


    
    def extend_initial_tree(self,
                            worldMap: MsgWorldMap,
                            altitude: float = None):
        
        connectedToEnd = False

        minDistanceToEnd = np.inf
        minDistanceIndex = 0

        while connectedToEnd is False:

            #generates a new point. we are experimenting with the growth method
            newPosition_candidate, newPosition_Cost, minCostParentIndex = self.getNewPosition_random(world_map=worldMap,
                                                                                           tree=self.tree,
                                                                                           segmentLength=self.segmentLength,
                                                                                           altitude=altitude)
            #obtains the parent position
            parentPosition = self.tree.getPosition(index=minCostParentIndex)

            #candidate sfc
            sfc_candidate = MsgFlightCorridor(primaryPosition=parentPosition,
                                              secondaryPosition=newPosition_candidate,
                                              primaryPosition_index=minCostParentIndex,
                                              numDimensions=self.numDimensions)

            #calls the function to check for intersection
            intersectionHappened = intersectionDetected(corridor=sfc_candidate,
                                                     world_map=worldMap)

            #if no intersection happened, add the canditdate to the tree
            if intersectionHappened is False:

                self.tree.add(position=newPosition_candidate,
                              parent=minCostParentIndex,
                              cost=newPosition_Cost,
                              connectsToGoal=False)
                
                self.tree.addSFC(sfc=sfc_candidate)

                #gets the index of the final node
                latestNode_index = self.tree.numPositions - 1


                #gets the vector from the new node to the end
                newNode_toEnd = self.endPosition - newPosition_candidate

                #gets the distance
                newNode_toEnd_distance = np.linalg.norm(newNode_toEnd)


                #if we are within the ranga
                if newNode_toEnd_distance < self.segmentLength:

                    #the cnadidate corridor to the end
                    sfc_candidate_newToEnd = MsgFlightCorridor(primaryPosition=newPosition_candidate,
                                                               secondaryPosition=self.endPosition,
                                                               primaryPosition_index=latestNode_index,
                                                               numDimensions=self.numDimensions)
                    
                    #calls the function to check for an intersection
                    intersectionHappened_end = intersectionDetected(corridor=sfc_candidate_newToEnd,
                                                                    world_map=worldMap)
                    

                    if intersectionHappened_end is False:
                        self.tree.connectsToGoal[-1] = True
                        self.tree.addSFC(sfc_candidate_newToEnd)

                        #sets connected to end to true
                        connectedToEnd = True


        return connectedToEnd    
    


    
    
    #get the new position (along with its marginal cost and parent Index)
    #by randomization processes
    def getNewPosition_random(self,
                              world_map: MsgWorldMap,
                              tree: MsgWaypoints_SFC,
                              segmentLength: float,
                              altitude: float = None):
        
        #gets the random position (2D or 3D based on information from the map message)
        random_position = randomPosition(world_map=world_map,
                                         altitude=altitude)

        #gets the tree positions
        allTreePositions = tree.getAllPositions()
        #concatenates together to get the array version of the all tree positions
        allTreePositions_array = np.concatenate(allTreePositions, axis=1)
        #creates a matrix of vectors defining the distances from all positions in the tree
        #out to the current random position
        distance_vectors_list = allTreePositions_array - np.tile(random_position, (1, tree.numPositions))
        #gets the list of distances
        distances_list = np.diag(distance_vectors_list.T @ distance_vectors_list)
        #gets the min distance index
        minCostParentIndex = np.argmin(distances_list)
        #gets the min distance squared
        minDistanceSquared = distances_list.item(minCostParentIndex)
        #gets the min distance
        minDistance = np.sqrt(minDistanceSquared)
        #gets the length, which is the minimum of the square root of minDistance and the segment length
        #we do this to choose the shortest of the two, if the distance is greater than the maximum acceptable
        #segment length. Though we may change this in the future.
        L = np.min([minDistance, segmentLength])
        #gets the cost value associated with the nearest node
        nearestNodeCost = tree.getCost(index=minCostParentIndex)

        #sets the current Node cost
        newPositionCost = nearestNodeCost + L

        #gets the previous position, based on the minimum cost constraint
        previousPosition = tree.getPosition(index=minCostParentIndex)

        #gets the cevtor from the closest tree node to the current 
        prevToRandomSample = random_position - previousPosition
        #gets the unit vector of the previous node to the current random sampled node
        prevToRandomSample_unit = prevToRandomSample / np.linalg.norm(prevToRandomSample)

        #gets the new position
        newPosition = previousPosition + L *prevToRandomSample_unit

        #returns the new position
        return newPosition, newPositionCost, minCostParentIndex
    

    #defines the function to get a new position, but this time, it is created by growing the current tree
    #by extending off of the currently existing nodes
    def getNewPosition_growth(self,
                              startPosition: np.ndarray,
                              endPosition: np.ndarray,
                              world_map: MsgWorldMap):

        #gets the number of indices in the tree
        numTreeIndices = self.tree.numPositions

        #gets the random index from which to grow the thing
        randomizedIndex = random.randint(a=0, b=(numTreeIndices-1))

        #gets the randomized index's position
        randomizedIndex_position = self.tree.getPosition(index=randomizedIndex)

        #gets the randomized index parent position
        randomizedIndex_parentIndex = self.tree.getParent(index=randomizedIndex)

        #gets the vector from the 


        pass



        


def randomPosition(world_map: MsgWorldMap,
                   altitude: float = None):
    
    #gets the number of dimensions
    numDimensions = world_map.numDimensions

    if numDimensions == 2:
        randomPosition = randomPosition_2D(world_map=world_map,
                                           altitude=altitude)
    elif numDimensions == 3:
        randomPosition = randomPosition_3D(world_map=world_map)

    #returns the random position
    return randomPosition

def randomPosition_2D(world_map: MsgWorldMap,
                      altitude: float):

    #gets the random position
    pn_rand = np.random.uniform(low=0.0, high=PLAN.city_width)
    pe_rand = np.random.uniform(low=0.0, high=PLAN.city_width)
    randomPosition = np.array([[pn_rand],[pe_rand],[-altitude]])

    return randomPosition


#defines the function to obtain a random position
def randomPosition_3D(world_map: MsgWorldMap):
    
    #gets the random position
    pn_rand = np.random.uniform(low=0.0, high=PLAN.city_width)
    pe_rand = np.random.uniform(low=0.0, high=PLAN.city_width)
    pd_rand = np.random.uniform(low=-PLAN.city_width, high=0.0)
    randomPosition = np.array([[pn_rand],[pe_rand],[pd_rand]])
    #returns the random position
    return randomPosition



#defines the function to get the intersection with the sfc candidate and the world map
def intersectionDetected(corridor: MsgFlightCorridor,
                         world_map: MsgWorldMap):
    

    #gets the map obstacles list
    map_A_b_lists = world_map.getAbMatricesLists()

    #gets the corridor A and b matrices
    A_corridor, b_corridor = corridor.getAbMatrices()
    
    intersection = False

    #iterates over all of the A b matrices
    for AbMatrices in map_A_b_lists:

        tempObstacleA = AbMatrices[0]
        tempObstacle_b = AbMatrices[1]

        #with the obstacle A and b, we check for intersection
        tempObstacleIntersectionOccurred = intersectionOccurred(A1=A_corridor,
                                                                b1=b_corridor,
                                                                A2=tempObstacleA,
                                                                b2=tempObstacle_b)
        
        intersection = intersection and tempObstacleIntersectionOccurred

        #if we find an intersection, then we return true
        if intersection:
            return intersection
        


    #if we don't find an intersection, we return false
    return intersection
    





#defines the function to smooth the path
def flight_corridors_smooth_path(waypoints_not_smooth: MsgWaypoints_SFC_3D,
                                 world_map: MsgWorldMap3D)->MsgWaypoints_SFC_3D:
    
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
    for (startNode_index, stopNode_index), tempCost, corridor\
          in zip(validIndices_list, cost_list, validFlightCorridors_list):
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
            potato = 0
    potato = 0
