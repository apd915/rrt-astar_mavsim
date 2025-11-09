#implements some tools for operating the safe flight corridors

from rrt_mavsim.message_types.msg_safeFlightCorridor import Msg_SFC
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
import time
import numpy as np
from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN


#given a list of safe flight corridors, and the generally desired spacing between control points on that flight corridor,
# we need a function that generates the list of number of allocated control points for each section of the spline.
def getNumCntPts_list(waypoints: MsgWaypoints_SFC,
                      numPointsPerUnit: float,
                      degree: int = FLIGHT_PLAN.degree)->list[int]:
    

    #gets the sfc list
    flight_corridor_list = waypoints.getAllFlightCorridors()

    numCntPts_list = []

    for flightCorridor in flight_corridor_list:

        #gets the sfc length
        flightCorridorLength = flightCorridor.getFlightCorridorLength()

        #gets the number of control points and adds 1 for good measure
        numControlPoints_temp = int(flightCorridorLength*numPointsPerUnit)

        #if the number of control points is less than 2*degree, we make it 2*degree
        if numControlPoints_temp < int(2*degree):

            numControlPoints_temp = int(2*degree)

        #appends to the list
        numCntPts_list.append(numControlPoints_temp)

    return numCntPts_list




#defines the function to get the initial and final control Points
#That is, to get the very first and very last three control points
#for the entire spline
#this is all completed in the relative frame of the waypoints (2D or 3D). To be converted to 3D
#this must be done later by each individual part
def getInitialFinalControlPoints(corridorWaypoints: MsgWaypoints_SFC,
                                 Va: float,
                                 degree: int,
                                 M: int):
    
    #gets the number of dimensions planned for the corridor waypoints
    numDimensions = corridorWaypoints.numDimensions
    
    #gets the positions
    positions = corridorWaypoints.getAllPositions()
    #gets the startPosition
    startPosition = positions[0]
    #gets the second position
    secondPosition = positions[1]
    #gets the second to last position
    secondToLastPosition = positions[-2]
    #gets the end position
    endPosition = positions[-1]
    #gets the startVelocity vector
    startVelocity_unit = (secondPosition - startPosition)
    startVelocity_unit = startVelocity_unit / np.linalg.norm(startVelocity_unit)
    startVelocity = startVelocity_unit * Va
    #gets the end velocity vector
    endVelocity_unit = (endPosition - secondToLastPosition)
    endVelocity_unit = endVelocity_unit / np.linalg.norm(endVelocity_unit)
    endVelocity = endVelocity_unit * Va
    startAccel = np.zeros((numDimensions, 1))
    endAccel = np.zeros((numDimensions, 1))
    #plugs these into the bspline parameters conditions
    startConditions = [startPosition, startVelocity, startAccel]
    
    endConditions = [endPosition, endVelocity, endAccel]

    #creates a static flight path object to generate the localized control points
    staticFlightPath_generator = staticFlightPath()
    
    #####
    #gets the localized control points for start and end
    controlPoints_start =\
          staticFlightPath_generator.getLocalizedControlPoints(conditions=startConditions,
                                                                    d=degree,
                                                                    M=M)
    #####
    controlPoints_end =\
          staticFlightPath_generator.getLocalizedControlPoints(conditions=endConditions,
                                                                    d=degree,
                                                                    M=M)

    return controlPoints_start, controlPoints_end



#defines the function to get the total number of control points
def getNumCntPts(numCntPts_list: list[int],
                 degree: int):

    #get the initial sum
    initialSum = sum(numCntPts_list)

    #gets the number of corridors, which is the number of items in the list
    numCorridors = len(numCntPts_list)

    #gets the num control points
    numCntPts = initialSum - degree * (numCorridors - 1)

    #returns the total number of contorl points
    return numCntPts