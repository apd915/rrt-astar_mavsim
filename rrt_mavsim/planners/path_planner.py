import rrt_mavsim.parameters.planner_parameters as PLAN_PARAM
import numpy as np
from rrt_mavsim.message_types.msg_bspline_conditions import MsgBsplineConditions
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC


#creates the Path Planner class
class PathPlanner:

    #creates the initialization function
    def __init__(self,
                 numDimensions: int,
                 altitude: float = PLAN_PARAM.altitude):
        
        self.numDimensions = numDimensions

        self.altitude = altitude
        
        #creates a waypoints message class
        self.waypoints = MsgWaypoints_SFC()

        #creates the rrt B-Spline creator function
        self.rrt_bspline = 
