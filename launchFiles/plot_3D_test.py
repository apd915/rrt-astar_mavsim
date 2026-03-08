import os, sys
import numpy as np
from eVTOL_BSplines.message_types.msg_sfc import Msg_SFC
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, FloatingBlocksParams, MapTypes
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.floatingBlocks_parameters as FLOATING_PARAM
import rrt_mavsim.parameters.aerosonde_parameters as AERO
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT
from rrt_mavsim.viewers.view_manager import ViewManager
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
from rrt_mavsim.viewers.plot_map_path import PlotMapPath
from rrt_mavsim.planners.bspline_generator import ObjectiveTypes
from bsplinegenerator.bsplines import BsplineEvaluation
import matplotlib.pyplot as plt
import time
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC

#creates a single waypoint message

tempSFC = MsgFlightCorridor(numDimensions=3,
                            primaryPosition=np.array([[0.0],[0.0],[0.0]]),
                            secondaryPosition=np.array([[700.0],[0.0],[0.0]]))

waypoints = MsgWaypoints_SFC(numDimensions=3)
waypoints.addSFC(sfc=tempSFC)

worldMap = MsgWorldMap(
    obstacleFieldType=MapTypes.FLOATING_BLOCKS,
    numDimensions_algorithm=FLOATING_PARAM.numDimensions,
    floatingBlocksParams=FloatingBlocksParams()
)

plotter = PlotMapPath(map=worldMap,
                      waypoints_smooth=waypoints)

plotter.plot(
    x_limits=FLOATING_PARAM.x_limits,
    y_limits=FLOATING_PARAM.y_limits,
    z_limits=FLOATING_PARAM.z_limits,
    aspectRatio=FLOATING_PARAM.aspect_ratio,
)

testPoint = 0
