#the rrt function for generating B-Splines
import numpy as np
from planners.bspline_parameters import BsplineParameters
from message_types.msg_bspline_conditions import MsgBsplineConditions
from message_types.msg_world_map import MsgWorldMap
from message_types.msg_waypoints import MsgWaypoints_SFC
from message_types.msg_flight_corridors import MsgFlightCorridor
import parameters.planner_parameters as PLAN
import parameters.flightCorridor_parameters as FLIGHT_PLAN
import random
import time
import scipy as sp
from tools.intersections import intersectionOccurred_Matrix, intersectionDetected
from tools.pathOptimization import findMinimumPath
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
                      segmentLength: float,
                      altitude: float):
        
        if self.numDimensions == 2:
            self.startPosition = FLIGHT_PLAN.initialPosition_2D
            self.endPosition = FLIGHT_PLAN.finalPosition_2D
        elif self.numDimensions == 3:
            self.startPosition = FLIGHT_PLAN.initialPosition_3D
            self.endPosition = FLIGHT_PLAN.finalPosition_3D

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


        #gets the waypoints not smooth from the list
        self.waypoints_not_smooth = findMinimumPath(tree=self.tree,
                                                    endPosition=self.endPosition)
        
        return self.waypoints_not_smooth

    def extend_initial_tree(self,
                            worldMap: MsgWorldMap,
                            altitude: float = None):
        
        connectedToEnd = False

        minDistanceToEnd = np.inf
        minDistanceIndex = 0

        intersectionHappened_list = []

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
            
            intersectionHappened_list.append(intersectionHappened)

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
                    intersectionHappened_list.append(intersectionHappened_end)
                    

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

