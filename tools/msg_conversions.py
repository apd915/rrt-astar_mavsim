from message_types.msg_flight_corridors import MsgFlightCorridor
from message_types.msg_waypoints import MsgWaypoints
import copy


from eVTOL_BSplines.submodules.path_generator.path_generation.path_generator import PathGenerator
from eVTOL_BSplines.submodules.path_generator.path_generation.safe_flight_corridor import SFC, SFC_Data, plot_sfcs, get2DRotationAndTranslationFromPoints
from eVTOL_BSplines.submodules.path_generator.path_generation.obstacle import Obstacle, plot_2D_obstacles
from eVTOL_BSplines.submodules.path_generator.path_generation.waypoint_data import plot2D_waypoints
from eVTOL_BSplines.submodules.path_generator.path_generation.waypoint_data import Waypoint, WaypointData
from eVTOL_BSplines.submodules.path_generator.path_generation.path_plotter import set_axes_equal

import numpy as np


#defines the function to convert from a list of flight corridors to sfc data output
#So, this creates the boundaries, in essence
def Corridors_to_pathGen_SFC(waypoints: MsgWaypoints)->SFC_Data:


    #gets the corridor list and the position list from the waypoints message
    positionsList = waypoints.getAllPositionsList()
    corridorList = waypoints.getCorridorList()

    #gets the dimensionality of the positions
    positionDimension = np.size(positionsList[0])

    #concatenates together the positions in the position list
    positionArray = np.ndarray((positionDimension,0))

    for position in positionsList:
        #appends to the positions list
        positionArray = np.concatenate((positionArray, position), axis=1)

    #creates the list of safe flight corridors from the path generator class
    pathGen_SFC_list = []

    #iterates over all the elements of the SFC list
    for corridor in corridorList:

        #while we itereate over all of the corridors, and get the stuff for it
        current_dim, current_rot_CL_to_world, current_trans = corridor.get_dim_rot_trans()

        #gets the vertices and normal vectors
        vertices = corridor.getVerticesList()
        normalVectors = corridor.getNormalVectorsList()

        #creates the safe flight corridor SFC
        path_gen_SFC = copy.deepcopy(SFC(dimensions=current_dim,
                                         translation=current_trans,
                                         rotation=current_rot_CL_to_world))
        
        #sets the vertices and normalVectors for this
        path_gen_SFC.setNormalsVertices_old(vertices=vertices,
                                            normalVectors=normalVectors)
        
        #appends to the path generator sfc list
        pathGen_SFC_list.append(path_gen_SFC)

    #now with the sfc list and the positions array, we get the sfc data

    sfc_data_output = SFC_Data(sfc_list=pathGen_SFC_list,
                               point_sequence=positionArray)
    
    #returns the sfc data output
    return sfc_data_output


#creates the function to convert between my version of waypoints to pathGen Waypoints
#so, this creates the start and stop position and velocities, in essence
def waypoints_to_pathGen_waypoints(waypoints: MsgWaypoints,
                                   Va: float = 25.0,
                                   numDimensions: int = 2)->WaypointData:
    
    #gets the flight corridors from the waypoints
    corridorList = waypoints.getCorridorList()

    #gets the first and the last corridors in the above list
    start_corridor = corridorList[0]
    corridorList_len = len(corridorList)
    end_corridor = corridorList[corridorList_len - 1]


    #gets the initial position (from the primary corridor)
    initial_position = start_corridor.get_primary_position()

    #gets the final position (from the final corridor)
    final_position = end_corridor.get_secondary_position()

    #reshapes the initial and final positions to be the correct number of dimensions
    initial_position = initial_position[:numDimensions, 0:1]
    final_position = final_position[:numDimensions, 0:1]


    #gets the start velocity direction
    start_velocity_direction = start_corridor.get_primary_to_secondary_unit()
    #gets the end velocity direction
    end_velocity_direction = end_corridor.get_primary_to_secondary_unit()

    #gets the start and end velocity
    start_velocity = start_velocity_direction * Va
    end_velocity = end_velocity_direction * Va

    #modifies the velocities so they are the correct number of dimensions
    start_velocity = start_velocity[:numDimensions, 0:1]
    end_velocity = end_velocity[:numDimensions, 0:1]

    #gets the waypoint for start and finish
    waypoint_pathGen_start = Waypoint(location=initial_position,
                                      velocity=start_velocity)
    waypoint_pathGen_end = Waypoint(location=final_position,
                                    velocity=end_velocity)
    
    #creates the waypoint data pathgen
    waypoint_data_pathGen = WaypointData(start_waypoint=waypoint_pathGen_start,
                                         end_waypoint=waypoint_pathGen_end)
    
    #returns the waypoint data pathgen
    return waypoint_data_pathGen



#creates a function to convert a 3d waypoint message to a 2d equivalent one.
def waypoints_3d_to_2d(waypoints_3d: MsgWaypoints):

    #creates the waypionts_2d variable
    waypoints_2d = MsgWaypoints(type='flight_corridor')

    #converts the 3d positions to 2d positions by omiting the final component
    positions_3d = waypoints_3d.positions

    #gets the 2d positions
    positions_2d = positions_3d[0:2,:]

    #sets the positions 2d into the new waypoints function
    waypoints_2d.positions = positions_2d

    #gets the list of 3d flight corridors
    flightCorridor_list_3d = waypoints_3d.flightCorridors

    #creates the list of 2d flight corridors
    flightCorridor_list_2d = []

    for flightCorridor_3d in flightCorridor_list_3d:

        flightCorridor_2d = flightCorridor_3d.convert_3d_to_2d()

        #appends to the flight corridor list
        flightCorridor_list_2d.append(flightCorridor_2d)

    #sets the 2d flight corridors
    waypoints_2d.flightCorridors = flightCorridor_list_2d

    return waypoints_2d



#creates a function to expand the original flight corridors
def expandStartEndCorridors(waypoints: MsgWaypoints,
                            startCntPts: np.ndarray,
                            endCntPts: np.ndarray):

    potato = 0