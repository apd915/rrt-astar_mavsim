import os, sys
import numpy as np
from eVTOL_BSplines.message_types.msg_sfc import Msg_SFC
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, CityParams, MapTypes 
import rrt_mavsim.parameters.city_parameters as CITY
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.aerosonde_parameters as AERO
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT
from rrt_mavsim.viewers.view_manager import ViewManager
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline

viewer = ViewManager()

#gets the altitude
altitude = CITY.altitude

#creates the plane message

startPosition_2D = CITY.startPosition_2D
endPosition_2D = CITY.endPosition_2D

path_gen = RRT_SFC_BSpline(numDimensions=CITY.numDimensions,
                           degree=FLIGHT.degree,
                           M=FLIGHT.M,
                           Va=PLAN.Va0,
                           rho=FLIGHT.rho,
                           step_length=FLIGHT.segmentLength,
                           numDesiredInitPaths=FLIGHT.numInitialPaths,
                           plane=CITY.plane_msg)



#creates the city parameters
city_params = CityParams(plane=CITY.plane_msg)
worldMap = MsgWorldMap(obstacleFieldType=MapTypes.CITY,
                       numDimensions_algorithm=CITY.numDimensions,
                       cityParams=city_params)

viewer.drawMap(world_map=worldMap)


path_gen.generateSFCPaths(startPosition_3D=CITY.startPosition_3D,
                          endPosition_3D=CITY.endPosition_3D,
                          worldMap=worldMap,
                          segmentLength=FLIGHT.segmentLength)

waypoints_not_smooth = path_gen.getWaypointsNotSmooth()
tree = path_gen.getTree()
controlPoints = path_gen.generateControlPoints(waypoints=waypoints_not_smooth,
                                               numPointsPerUnit=int(FLIGHT.numPoints_perUnit))

viewer.update_planning_tree(waypoints=waypoints_not_smooth,
                            tree=tree,
                            world_map=worldMap,
                            optimizedControlPoints=controlPoints,
                            plane=CITY.plane_msg)

testPoint = 0
