import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
import parameters.planarVTOL_map_parameters as VTOL_PARAM

from message_types.msg_world_map import MsgWorldMap, PlanarVTOLParams, MapTypes





p0 = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

#initializes the parameters
vtolParam = PlanarVTOLParams(fieldHeight=VTOL_PARAM.mapHeight,
                             fieldLength=VTOL_PARAM.mapLength,
                             obstacleMaxWidth=VTOL_PARAM.obstacleMaxWidth,
                             obstacleMinWidth=VTOL_PARAM.obstacleMinWidth,
                             obstacleDepth=VTOL_PARAM.obstacleDepth,
                             mapOrigin_2D=np.array([[0.0],[0.0]]),
                             mapOrigin_3D=p0,
                             n_hat=n_hat,
                             numObstacles=VTOL_PARAM.numObstacles)


typeOfMap = MapTypes.PLANAR_VTOL

worldMap = MsgWorldMap(obstacleFieldType=typeOfMap,
                       numDimensions_algorithm=2,
                       planarVTOL_Params=vtolParam)