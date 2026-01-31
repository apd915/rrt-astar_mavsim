import os, sys
import numpy as np
from eVTOL_BSplines.message_types.msg_sfc import Msg_SFC
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, CityParams, MapTypes 
import rrt_mavsim.parameters.city_parameters as CITY_PARAM
from rrt_mavsim.viewers.view_manager import ViewManager



#gets the altitude
altitude = CITY_PARAM.altitude

#creates the plane message
mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[-altitude]])
n_hat = np.array([[0.0],[0.0],[1.0]])

plane_msg = MsgPlane(n_hat=n_hat,origin_3D=mapOrigin_3D)

startPosition_2D = CITY_PARAM.startPosition_2D
endPosition_2D = CITY_PARAM.endPosition_2D

#creates the city parameters
city_params = CityParams(plane=plane_msg)

worldMap = MsgWorldMap(obstacleFieldType=MapTypes.CITY,
                       numDimensions_algorithm=CITY_PARAM.numDimensions,
                       cityParams=city_params)
viewer = ViewManager()


testPoint = 0
