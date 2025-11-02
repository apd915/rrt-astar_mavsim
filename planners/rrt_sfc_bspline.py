import numpy as np
from planners.bspline_parameters import BsplineParameters
from message_types.msg_bspline_conditions import MsgBsplineConditions
from message_types.msg_world_map import MsgWorldMap, MapTypes, PlanarVTOLParams
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
                      startPosition: np.ndarray,
                      endPosition: np.ndarray,
                      worldMap: MsgWorldMap,
                      segmentLength: float):
        
        self.startPosition = startPosition
        self.endPosition = endPosition
        self.worldMap = worldMap
        self.segmentLength = segmentLength

        self.tree = MsgWaypoints_SFC(numDimensions=self.numDimensions)


        #calls the generate paths based on the 2D or 3D case
        if self.numDimensions == 2:

            self.__generatePaths_2D()

        elif self.numDimensions == 3:

            self.__generatePaths_3D()


    #creates the version for 2D
    def __generatePaths_2D(self):

        #gets the projected start and end positions
        startPosition_projected = projectPosition(pos_3D=self.startPosition,
                                                  p_0=self.p0,
                                                  n_hat=self.n_hat)
        
        endPosition_projected = projectPosition(pos_3D=self.endPosition,
                                                p_0=self.p0,
                                                n_hat=self.n_hat)

        potato = 0

    def __generatePaths_3D(self):

        pass



#defines the function to project a position onto a plane
def projectPosition(pos_3D: np.ndarray, #the 3D position of the point (the point not on the plane)
                    p_0: np.ndarray, #the 3D position of the origin on the plane (The zero position on the plane)
                    n_hat: np.ndarray): #the unit vector of the plane 
    
    #gets the vector from p0 to pos_3D
    vec_p0_to_point = pos_3D - p_0

    dotProduct = np.dot(a=vec_p0_to_point.flatten(), b=n_hat.flatten())
    #now that this is relative to the origin, gets the projection onto the n_hat
    n_hat_proj = (dotProduct/(np.linalg.norm(n_hat)**2))*n_hat
    #gets the projection onto the plane, which is the vector minus the n_hat projection
    vec_proj = vec_p0_to_point - n_hat_proj
    #returns the projected vector
    return vec_proj