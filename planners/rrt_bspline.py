#the rrt function for generating B-Splines
import numpy as np
from planners.bspline_parameters import BsplineParameters
from message_types.msg_bspline_conditions import MsgBsplineConditions
from message_types.msg_world_map import MsgWorldMap


#creates the RRT B-Spline class
class RRTBSpline:

    #creates the init function
    def __init__(self,
                 numDimensions: int,
                 M: int,
                 Va: float,
                 rho: np.ndarray,
                 step_length: float):
        
        #saves all of them
        self.numDimensions = numDimensions
        self.M = M
        self.Va = Va
        self.rho = rho
        self.stepLength = step_length


        #creates the B-Splines
        self.bsplineParam = BsplineParameters()

    #creates the update function
    def update(self,
               start_conditions: MsgBsplineConditions,
               end_conditions: MsgBsplineConditions,
               world_map: MsgWorldMap):

        pass