import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


import numpy as np
from message_types.msg_world_map import MsgWorldMap, PlanarVTOLParams, MapTypes
from viewers.view_manager import ViewManager
from planners.rrt_sfc_bspline import RRT_SFC_BSpline
import parameters.planner_parameters as PLAN
import parameters.flightCorridor_parameters as FLIGHT_PLAN
import parameters.planarVTOL_map_parameters as VTOL_PARAM


numDimensions = 2

viewerManager = ViewManager(mav=False,
                            planningFlag=True)

mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])


#creates the planar vtol params
params = PlanarVTOLParams(mapOrigin_2D=mapOrigin_2D,
                          mapOrigin_3D=mapOrigin_3D,
                          n_hat=n_hat)

startPosition = params.startPosition
endPosition = params.endPosition

worldMap = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL,
                       numDimensions_algorithm=numDimensions,
                       planarVTOL_Params=params)


planner = RRT_SFC_BSpline(numDimensions=numDimensions,
                          M=FLIGHT_PLAN.M,
                          Va=PLAN.Va0,
                          rho=FLIGHT_PLAN.rho,
                          step_length=FLIGHT_PLAN.segmentLength,
                          numDesiredInitPaths=FLIGHT_PLAN.numInitialPaths,
                          n_hat=n_hat,
                          p0=mapOrigin_3D)


planner.generatePaths(startPosition_3D=startPosition,
                      endPosition_3D=endPosition,
                      worldMap=worldMap,
                      segmentLength=FLIGHT_PLAN.segmentLength)

potato = 0