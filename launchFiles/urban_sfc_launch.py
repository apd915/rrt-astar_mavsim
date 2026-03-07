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
from rrt_mavsim.viewers.plot_map_path import PlotMapPath
from rrt_mavsim.planners.bspline_generator import ObjectiveTypes
import matplotlib.pyplot as plt

viewer = ViewManager()



# gets the altitude
altitude = CITY.altitude

# creates the plane message

startPosition_2D = CITY.startPosition_2D
endPosition_2D = CITY.endPosition_2D

path_gen = RRT_SFC_BSpline(
    numDimensions=CITY.numDimensions,
    degree=FLIGHT.degree,
    M=FLIGHT.M,
    Va=PLAN.Va0,
    rho=FLIGHT.rho,
    step_length=FLIGHT.segmentLength,
    numDesiredInitPaths=FLIGHT.numInitialPaths,
    plane=CITY.plane_msg,
    chiMax=FLIGHT.Chi_max,
)

# creates the city parameters
city_params = CityParams(plane=CITY.plane_msg)
worldMap = MsgWorldMap(
    obstacleFieldType=MapTypes.CITY,
    numDimensions_algorithm=CITY.numDimensions,
    cityParams=city_params,
)

viewer.drawMap(world_map=worldMap)

path_gen.generateSFCPaths(
    startPosition_3D=CITY.startPosition_3D,
    endPosition_3D=CITY.endPosition_3D,
    worldMap=worldMap,
    segmentLength=FLIGHT.segmentLength,
)

waypoints_not_smooth = path_gen.getWaypointsNotSmooth()
waypoints_smooth = path_gen.getWaypointsSmooth()

tree = path_gen.getTree()

controlPoints_minDistance_notSmooth = path_gen.generateControlPoints(
    waypoints=waypoints_not_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_DISTANCE
)
controlPoints_minVelocity_notSmooth = path_gen.generateControlPoints(
    waypoints=waypoints_not_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_VELOCITY
)
controlPoints_minAccel_notSmooth = path_gen.generateControlPoints(
    waypoints=waypoints_not_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_ACCELERATION
)

controlPoints_minDistance_smooth = path_gen.generateControlPoints(
    waypoints=waypoints_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_DISTANCE
)
controlPoints_minVelocity_smooth = path_gen.generateControlPoints(
    waypoints=waypoints_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_VELOCITY
)
controlPoints_minAccel_smooth = path_gen.generateControlPoints(
    waypoints=waypoints_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_ACCELERATION
)

controlPointsList = [controlPoints_minDistance_smooth,
                     controlPoints_minVelocity_smooth,
                     controlPoints_minAccel_smooth]

#plots the not smooth control points
viewer.update_planning_tree(
    waypoints_not_smooth=waypoints_not_smooth,
    waypoints_smooth=waypoints_smooth,
    tree=tree,
    degree=CITY.degree,
    numdimensions=CITY.numDimensions,
    R_ned_to_alt=CITY.R,
    world_map=worldMap,
    controlPoints_list=controlPointsList,
    plane=CITY.plane_msg,
)

plotter = PlotMapPath(
    map=worldMap,
    waypoints_not_smooth=waypoints_not_smooth,
    waypoints_smooth=waypoints_smooth,
    controlPoints_not_smooth_list=None,
    controlPoints_smooth_list=controlPointsList,
    plane=CITY.plane_msg,
)

plotter.plot(
    x_limits=CITY.x_limits,
    y_limits=CITY.y_limits,
    z_limits=CITY.z_limits,
    aspectRatio=CITY.aspect_ratio,
)

testPoint = 0
