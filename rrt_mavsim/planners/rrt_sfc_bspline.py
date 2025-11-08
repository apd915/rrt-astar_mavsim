import numpy as np
from rrt_mavsim.planners.bspline_parameters import BsplineParameters
from rrt_mavsim.message_types.msg_bspline_conditions import MsgBsplineConditions
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, MapTypes, PlanarVTOLParams
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
import random
import time
import scipy as sp
from rrt_mavsim.tools.intersections import intersectionOccurred_Matrix, intersectionDetected
from rrt_mavsim.tools.pathOptimization import findMinimumPath
from rrt_mavsim.tools.plane_projections import projectPosition_toPlane, map_2D_to_3D, map_3D_to_2D
import heapq


class RRT_SFC_BSpline:


    #creates the init function
    def __init__(self,
                 numDimensions: int,
                 M: int,
                 Va: float,
                 rho: np.ndarray,
                 step_length: float,
                 numDesiredInitPaths: int,
                 n_hat: np.ndarray = None,#the normal vector for the working plane (if )
                 p0: np.ndarray = None):
        
        #saves all of them
        self.numDimensions = numDimensions
        self.M = M
        self.Va = Va
        self.rho = rho
        self.stepLength = step_length
        self.numDesiredInitPaths = numDesiredInitPaths

        self.n_hat = n_hat
        self.p0 = p0

        #creates the B-Splines
        self.bsplineParam = BsplineParameters()


    def generatePaths(self,
                      startPosition_3D: np.ndarray,
                      endPosition_3D: np.ndarray,
                      worldMap: MsgWorldMap,
                      segmentLength: float):
        
        self.startPosition_3D = startPosition_3D
        self.endPosition_3D = endPosition_3D
        self.worldMap = worldMap
        self.segmentLength = segmentLength

        self.tree = MsgWaypoints_SFC(numDimensions=self.numDimensions)


        #calls the generate paths based on the 2D or 3D case
        #saves the changes and the updates to the tree
        if self.numDimensions == 2:

            self.__generatePaths_2D()

        elif self.numDimensions == 3:

            self.__generatePaths_3D()


    #creates the version for 2D
    def __generatePaths_2D(self):

        #gets the projected start and end positions.
        #that is, they are still in 3D, but they represent the 3D position
        #that will need to be projected onto the workplane
        startPosition_3D_projected = projectPosition_toPlane(pos_3D=self.startPosition_3D,
                                                     p_0=self.p0,
                                                     n_hat=self.n_hat)
        
        endPosition_3D_projected = projectPosition_toPlane(pos_3D=self.endPosition_3D,
                                                   p_0=self.p0,
                                                   n_hat=self.n_hat)
        
        
        #gets the start position 2D and same for the end position
        startPosition_2D = map_3D_to_2D(pos_3D=startPosition_3D_projected,
                                        n_hat=self.n_hat,
                                        p0=self.p0)
        
        endPosition_2D = map_3D_to_2D(pos_3D=endPosition_3D_projected,
                                      n_hat=self.n_hat,
                                      p0=self.p0)
        #
        #adds the projected start position
        self.tree.add(position=startPosition_2D,
                      parent=np.inf,
                      cost=0.0,
                      connectsToGoal=False)
        

        currentNumPathsFound = 0

        while currentNumPathsFound < self.numDesiredInitPaths:

            foundNewPathFlag = self.__extendInitialTree(endPosition=endPosition_2D)

            currentNumPathsFound += foundNewPathFlag

        #gets the waypoints not smooth from the list
        self.waypoints_not_smooth = findMinimumPath(tree=self.tree,
                                                    endPosition=endPosition_2D)
        
        #returns the not smooth waypoints
        return self.waypoints_not_smooth

    def __generatePaths_3D(self):
        
        #adds the tree position
        self.tree.add(position=self.startPosition_3D,
                      parent=np.inf,
                      cost=0.0,
                      connectsToGoal=False)
        

        currentNumPathsFound = 0

        while currentNumPathsFound < self.numDesiredInitPaths:

            #sets the found new path flag
            foundNewPathFlag = self.__extendInitialTree(endPosition=self.endPosition_3D)

            currentNumPathsFound += foundNewPathFlag

        
        #get the not smooth waypoints
        self.waypoints_not_smooth = findMinimumPath(tree=self.tree,
                                                    endPosition=self.endPosition_3D)
        
        #returns the not smooth waypoints
        return self.waypoints_not_smooth


    #the 2D case for extending the initial tree
    def __extendInitialTree(self,
                            endPosition: np.ndarray):
        
        #sets that it is not connected to the end yet
        connectedToEnd = False
        
        minDistanceToEnd = np.inf
        minDistanceIndex = 0

        intersectionHappened_list = []

        #iterates while it is not yet connected to the end
        while connectedToEnd is False:
            newPositionCandidate, newPositionCandidate_cost, candidate_minCostParentIndex =\
                generateRandomCandidate(world_map=self.worldMap,
                                        tree=self.tree,
                                        segmentLength=self.segmentLength)

            #obtains the parent position
            parentPosition = self.tree.getPosition(index=candidate_minCostParentIndex)

            #candidate sfc
            sfc_candidate = MsgFlightCorridor(primaryPosition=parentPosition,
                                              secondaryPosition=newPositionCandidate,
                                              primaryPosition_index=candidate_minCostParentIndex,
                                              numDimensions=self.numDimensions)

            #calls the function to check for intersection
            intersectionHappened = intersectionDetected(corridor=sfc_candidate,
                                                     world_map=self.worldMap)
            intersectionHappened_list.append(intersectionHappened)

            #if no intersection happened, add the canditdate to the tree
            if intersectionHappened is False:

                self.tree.add(position=newPositionCandidate,
                              parent=candidate_minCostParentIndex,
                              cost=newPositionCandidate_cost,
                              connectsToGoal=False)
                
                self.tree.addSFC(sfc=sfc_candidate)

                #gets the index of the final node
                latestNode_index = self.tree.numPositions - 1


                #gets the vector from the new node to the end
                newNode_toEnd = endPosition - newPositionCandidate

                #gets the distance
                newNode_toEnd_distance = np.linalg.norm(newNode_toEnd)


                #if we are within the ranga
                if newNode_toEnd_distance < self.segmentLength:

                    #the cnadidate corridor to the end
                    sfc_candidate_newToEnd = MsgFlightCorridor(primaryPosition=newPositionCandidate,
                                                               secondaryPosition=endPosition,
                                                               primaryPosition_index=latestNode_index,
                                                               numDimensions=self.numDimensions)
                    
                    #calls the function to check for an intersection
                    intersectionHappened_end = intersectionDetected(corridor=sfc_candidate_newToEnd,
                                                                    world_map=self.worldMap)
                    intersectionHappened_list.append(intersectionHappened_end)
                    

                    if intersectionHappened_end is False:
                        self.tree.connectsToGoal[-1] = True
                        self.tree.addSFC(sfc_candidate_newToEnd)

                        #sets connected to end to true
                        connectedToEnd = True
        
        
        return connectedToEnd
    
    def getWaypointsNotSmooth(self):
        return self.waypoints_not_smooth



#gets a random new position for a plane position
def generateRandomCandidate(world_map: MsgWorldMap,
                            tree: MsgWaypoints_SFC,
                            segmentLength: float):
    
    #case map is 2D
    if world_map.numDimensions_algorithm == 2:
        #gets a 2D random position here in the tree
        randomPosition = generateRandomPosition_2D(worldMap=world_map)
    elif world_map.numDimensions_algorithm == 3:
        #gets a 3D random position here in the tree
        randomPosition = generateRandomPosition_3D(worldMap=world_map)


    #gets the tree positions
    allTreePositions = tree.getAllPositions()
    #concatenates together to get the array version of the all tree positions
    allTreePositions_array = np.concatenate(allTreePositions, axis=1)
    #creates a matrix of vectors defining the distances from all positions in the tree
    #out to the current random position
    distance_vectors_list = allTreePositions_array - np.tile(randomPosition, (1, tree.numPositions))
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
    prevToRandomSample = randomPosition - previousPosition
    #gets the unit vector of the previous node to the current random sampled node
    prevToRandomSample_unit = prevToRandomSample / np.linalg.norm(prevToRandomSample)

    #gets the new position
    newPosition = previousPosition + L *prevToRandomSample_unit

    #returns the new position
    return newPosition, newPositionCost, minCostParentIndex




#generates the 2D position
def generateRandomPosition_2D(worldMap: MsgWorldMap):

    #gets the world map dimensions
    startDim = worldMap.searchDimensions_start
    endDim = worldMap.searchDimensions_end

    #gets the map dimensions rotated into 

    #gets the x_random position
    x_random = np.random.uniform(low=startDim.item(0), high=endDim.item(0))

    y_random = np.random.uniform(low=startDim.item(1), high=endDim.item(1))

    random_position = np.array([[x_random],[y_random]])

    return random_position


#generates the 3D position
def generateRandomPosition_3D(worldMap: MsgWorldMap):

    #gets the world map dimensions
    startDim = worldMap.searchDimensions_start
    endDim = worldMap.searchDimensions_end

    #gets the x_random position
    x_random = np.random.uniform(low=startDim.item(0), high=endDim.item(0))

    y_random = np.random.uniform(low=startDim.item(1), high=endDim.item(1))

    z_random = np.random.uniform(low=startDim.item(2), high=endDim.item(2))

    random_position = np.array([[x_random],[y_random],[z_random]])

    return random_position




