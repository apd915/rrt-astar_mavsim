#in this file, I attempt to implement wrapping david christensen's code in 
#this file and making it accessable to the kind of classes and obstacles, etc 
#that I am used to working with
from eVTOL_BSplines.submodules.path_generator.path_generation.path_generator import PathGenerator

from eVTOL_BSplines.submodules.path_generator.PathObjectivesAndConstraints.python_wrappers.objective_functions import ObjectiveFunctions
from eVTOL_BSplines.submodules.path_generator.PathObjectivesAndConstraints.python_wrappers.curvature_constraints import CurvatureConstraints
from eVTOL_BSplines.submodules.path_generator.PathObjectivesAndConstraints.python_wrappers.obstacle_constraints import ObstacleConstraints
from eVTOL_BSplines.submodules.path_generator.PathObjectivesAndConstraints.python_wrappers.incline_constraints import InclineConstraints
from eVTOL_BSplines.submodules.path_generator.PathObjectivesAndConstraints.python_wrappers.waypoint_constraints import WaypointConstraints
from eVTOL_BSplines.submodules.path_generator.path_generation.waypoint_data import Waypoint, WaypointData
from eVTOL_BSplines.submodules.path_generator.path_generation.obstacle import Obstacle

from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.tools.obstacle_conversions import *
from rrt_mavsim.message_types.msg_plane import MsgPlane
import numpy as np

from scipy.optimize import minimize, NonlinearConstraint



class PathOptimizer:


    def __init__(self,
                 numDimensions: int,
                 world_map: MsgWorldMap,
                 plane: MsgPlane,
                 num_intervals_free_space: int = 5,
                 degree: int = 3):
        
        self.numDimensions = numDimensions
        self.num_intervals_free_space = num_intervals_free_space
        self.degree = degree

        self.world_map = world_map

        #gets the circular obstacle list
        self.obstacles_circular = worldMap_rectToCircle_conversion(worldMap=self.world_map,
                                                                   plane=plane)

        #creates the obstacle constraints object
        self.obstacles_constraints_object = ObstacleConstraints(dimension=self.numDimensions)
        

        potato = 0


    #defines the function to optimize the spline from the circular obstacles constraints

    def generate_path(self,
                      controlPoints_RRTOutput: np.ndarray,
                      startPosition: np.ndarray,
                      endPosition: np.ndarray):
        
        #converts from start and end conditions to start and end waypoints
        startWaypoint = Waypoint(location=startPosition)
        
        endWaypoint = Waypoint(location=endPosition)

        #creates the waypoint data
        waypoints = WaypointData(start_waypoint=startWaypoint,
                                 end_waypoint=endWaypoint)
        
        #gets the starting and endind d control points (which need to stay fixed)

        startControlPoints_init = controlPoints_RRTOutput[:,:self.degree]
        endControlPoints_init = controlPoints_RRTOutput[:,(-self.degree):]

        #and the central partitions
        variableControlPoints_init = controlPoints_RRTOutput[:,self.degree:(-self.degree)]

        self.__create_constraints(obstacles_list=self.obstacles_circular)


        potato = 0


    #wrapper function to create all the constraints
    def __create_constraints(self,
                             obstacles_list: list[Obstacle]):
        
        if obstacles_list is not None:

            circularObstacleConstraints =\
                  self.__create_circular_obstacle_constraints(obstacles_list=obstacles_list)


    #creates the constraints for the obstacles which are scipy Nonlinear constraints
    def __create_circular_obstacle_constraints(self,
                                               obstacles_list: list[Obstacle]):
        

        for i, obstacle in enumerate(obstacles_list):


            
            potato = 0
