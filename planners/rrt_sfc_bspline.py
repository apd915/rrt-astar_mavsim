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


    def generatePaths(self,
                      startPosition: np.ndarray,
                      endPosition: np.ndarray,
                      worldMap: MsgWorldMap,
                      segmentLength: float,
                      altitude: float):
        
        self.startPosition = startPosition
        self.endPosition = endPosition
        self.worldMap = worldMap
        self.segmentLength = segmentLength
        self.altitude = altitude

        self.tree = MsgWaypoints_SFC