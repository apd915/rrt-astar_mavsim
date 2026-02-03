import os, sys

cwd = os.getcwd()
paths = sys.path

import numpy as np
from rrt_mavsim.message_types.msg_world_map import (
    MsgWorldMap,
    PlanarVTOLParams,
    MapTypes,
)
from rrt_mavsim.viewers.view_manager import ViewManager
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
import rrt_mavsim.parameters.planarVTOL_map_parameters as VTOL_PARAM
from rrt_mavsim.message_types.msg_plane import MsgPlane

numDimensions = 2

viewerManager = ViewManager(mav=False, planningFlag=True)

mapOrigin_2D = np.array([[0.0], [0.0]])
mapOrigin_3D = np.array([[0.0], [0.0], [0.0]])
n_hat = np.array([[0.0], [1.0], [0.0]])
# creates the plane message
planeMsg = MsgPlane(n_hat=n_hat, origin_3D=mapOrigin_3D)


# creates the planar vtol params
params = PlanarVTOLParams(
    mapOrigin_2D=mapOrigin_2D, mapOrigin_3D=mapOrigin_3D, n_hat=n_hat
)

startPosition = params.startPosition
endPosition = params.endPosition

worldMap = MsgWorldMap(
    obstacleFieldType=MapTypes.PLANAR_VTOL,
    numDimensions_algorithm=numDimensions,
    planarVTOL_Params=params,
)


planner = RRT_SFC_BSpline(
    numDimensions=numDimensions,
    M=FLIGHT_PLAN.M,
    degree=FLIGHT_PLAN.degree,
    Va=PLAN.Va0,
    rho=FLIGHT_PLAN.rho,
    step_length=FLIGHT_PLAN.segmentLength,
    numDesiredInitPaths=FLIGHT_PLAN.numInitialPaths,
    plane=planeMsg,
)


planner.generateSFCPaths(
    startPosition_3D=startPosition,
    endPosition_3D=endPosition,
    worldMap=worldMap,
    segmentLength=FLIGHT_PLAN.segmentLength,
)


# gets the not smooth waypoints
waypointsNotSmooth = planner.getWaypointsNotSmooth()
tree = planner.getTree()


controlPoints = planner.generateControlPoints(
    waypoints=waypointsNotSmooth, numPointsPerUnit=int(FLIGHT_PLAN.numPoints_perUnit)
)

viewerManager.update_planning_tree(
    waypoints_not_smooth=waypointsNotSmooth,
    tree=tree,
    world_map=worldMap,
    optimizedControlPoints=controlPoints,
)


potato = 0
