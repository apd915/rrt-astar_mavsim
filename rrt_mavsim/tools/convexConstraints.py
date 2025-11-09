#this file implements convexity and constraints for the safe flight corridors
import cvxpy as cvp
import numpy as np
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_safeFlightCorridor import Msg_SFC
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC





#defines the function to create overlapping control point constraints
def get_overlapping_control_points_constraints(self,
                                               controlPoints_var: cvp.Variable,
                                               waypointData: MsgWaypoints_SFC,
                                               numCntPts_list: list[int],
                                               startControlPoints: np.ndarray,
                                               endControlPoints: np.ndarray):
    
    #obstains the number of dimensions 
    num_dimensions = waypointData.numDimensions
    #gets the flight corridor list
    flightCorridorList = waypointData.getAllFlightCorridors()


    



    potato = 0



