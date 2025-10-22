#creates the launch file for the B-Spline rrt SFC launch

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from message_types.msg_world_map import MsgWorldMap
from viewers.view_manager import ViewManager
from planners.rrt_bspline import RRTBSpline
import parameters.planner_parameters as PLAN
import parameters.flightCorridor_parameters as FLIGHT_PLAN


#sets the number of dimensions right here
numDimensions = 3



viewerManager = ViewManager(mav=False,
                            planningFlag=True)


worldMap = MsgWorldMap(obstacleFieldType='rectangular',
                       numDimensions = 2,
                       fieldWidth=2000.0,
                       obstacleWidthRatio=0.5,
                       obstacleWidth_sigma=1.0,
                       altitude=100.0,
                       numBlocks=4)

pathGenerator = RRTBSpline(numDimensions=numDimensions,
                           M=FLIGHT_PLAN.M,
                           Va=PLAN.Va0,
                           rho=FLIGHT_PLAN.rho,
                           step_length=FLIGHT_PLAN.segmentLength,
                           numDesiredInitPaths=FLIGHT_PLAN.numInitialPaths)



#calls the function to generate the paths
pathGenerator.generatePaths(worldMap=worldMap,
                            startPosition=FLIGHT_PLAN.initialPosition,
                            endPosition=FLIGHT_PLAN.finalPosition,
                            segmentLength=FLIGHT_PLAN.segmentLength,
                            altitude=PLAN.altitude)



viewerManager.update_planning_tree(waypoints=None,
                                   waypoints_not_smooth=None,
                                   tree=None,
                                   world_map=worldMap,
                                   optimizedControlPoints=None)



potato = 0
