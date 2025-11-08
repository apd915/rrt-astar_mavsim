import numpy as np
import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


from rrt_mavsim.tools.smoothingTools import *
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, MapTypes, PlanarVTOLParams
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor


#sets up the planar VTOL params
params = PlanarVTOLParams(mapOrigin_2D=np.array([[0.0],[0.0]]),
                          mapOrigin_3D=np.array([[0.0],[0.0],[0.0]]),
                          n_hat=np.array([[0.0],[1.0],[0.0]]),
                          numObstacles=0)


#creates the list of positions
positions = [np.array([[0.0],[0.0]]),
             np.array([[1000.0],[0.0]]),
             np.array([[2000.0],[1000.0]])]


flightCorridors = []

for i in (range(len(positions)-1)):

    startPos = positions[i]
    endPos = positions[i+1]

    #gets the flight corridor for this
    flightCorridor = MsgFlightCorridor(numDimensions=2,
                                       primaryPosition=startPos,
                                       secondaryPosition=endPos,
                                       primaryPosition_index=i,
                                       secondaryPosition_index=(i+1))
    
    flightCorridors.append(flightCorridor)





#creates the waypoints
waypoints = MsgWaypoints_SFC(numDimensions=2)

waypoints.positions=positions
waypoints.flightCorridors = flightCorridors





#first, we need to generate a world map
worldMap = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL,
                       numDimensions_algorithm=2,
                       planarVTOL_Params=params)

waypoints_smooth = flight_corridors_smooth_path_new(waypoints_not_smooth=waypoints,
                                                    world_map=worldMap,
                                                    psi_min=np.pi/2)


#calls the 


potato = 0