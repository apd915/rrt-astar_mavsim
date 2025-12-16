import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.tools.intersections import intersectionDetected, intersectionOccurred_Matrix
import numpy as np
import rrt_mavsim.parameters.planner_parameters as PLAN


numDimensions = 2

worldMap = MsgWorldMap(obstacleFieldType='rectangular',
                       numDimensions = numDimensions,
                       fieldWidth=2000.0,
                       obstacleWidthRatio=0.3,
                       obstacleWidth_sigma=1.0,
                       numBlocks=5)

#gets the A and b matrices from the first obstacle
A_obs = (worldMap.Ab_3D_list[0])[0]
b_obs = (worldMap.Ab_3D_list[0])[1]

#creates a flight Corridor
tempFlightCorridor = MsgFlightCorridor(primaryPosition=np.array([[200.0],[0.0],[PLAN.altitude]]),
                                       secondaryPosition=np.array([[200.0],[200.0],[PLAN.altitude]]),
                                       numDimensions=numDimensions)

#gets the A and B matrices from the temp flight corridor
A_sfc, b_sfc = tempFlightCorridor.getAbMatrices()


#gets whether an intersection is feasible between the two
intersectFound = intersectionDetected(corridor=tempFlightCorridor,
                                      world_map=worldMap)

#function calls the direct matrix function
intersectFound_matrix = intersectionOccurred_Matrix(A1=A_obs,
                                                    b1=b_obs,
                                                    A2=A_sfc,
                                                    b2=b_sfc)

potato = 0