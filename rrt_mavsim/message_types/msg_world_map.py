
import rrt_mavsim.parameters.planner_parameters as PLAN
import numpy as np
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.planarVTOL_map_parameters as PLANAR_PARAM
from rrt_mavsim.tools.obstacles import RectangularObstacle
from scipy.optimize import linprog
import time
from enum import Enum
from rrt_mavsim.tools.getEulerUnit import getRotFromUnitVec
from rrt_mavsim.tools.plane_projections import *
from rrt_mavsim.message_types.msg_plane import MsgPlane


#creates the enumeration for 

class MapTypes(str, Enum):
    CITY = 'City'
    FLOATING_BLOCKS = 'FloatingBlocks'
    PLANAR_VTOL = 'PlanarVTOL'
    MAZE = 'Maze'
    PLANAR_VTOL_SIMPLIFIED = 'PlanarVTOLSimplified'


class PlanarVTOLParams:

    def __init__(self,
                 fieldLength: float = PLANAR_PARAM.mapLength,
                 fieldHeight: float = PLANAR_PARAM.mapHeight,
                 obstacleMaxWidth: float = PLANAR_PARAM.obstacleMaxWidth,
                 obstacleMinWidth: float = PLANAR_PARAM.obstacleMinWidth,
                 obstacleDepth: float = PLANAR_PARAM.obstacleDepth,
                 mapOrigin_2D: np.ndarray = None,
                 mapOrigin_3D: np.ndarray = None,
                 n_hat: np.ndarray = None,
                 numObstacles: int = PLANAR_PARAM.numObstacles):
        
        self.fieldLength = fieldLength
        self.fieldHeight = fieldHeight
        self.obstacleMaxWidth = obstacleMaxWidth
        self.obstacleMinWidth = obstacleMinWidth
        self.obstacleDepth = obstacleDepth
        self.mapOrigin_2D = mapOrigin_2D
        self.mapOrigin_3D = mapOrigin_3D# this is also called p0
        self.n_hat = n_hat
        self.numObstacles = numObstacles


        #creates the dimensions of search
        #Note on the end altitude. Remember that in the 3D world frame, we define the positive
        #Z axis in the down direction. But I want the plane to end up at an altitude of 2000 units
        #so that means the search area will be freom the origin (0,0) to (0,2k) to (10k, -2k) to (10k,0)
        self.searchDimensions_3D = np.array([[0.0, self.fieldLength],
                                             [0.0, 0.0],
                                             [0.0, -self.fieldHeight]])
        
        self.startPosition = np.array([[0.0],[0.0],[0.0]])
        self.endPosition = np.array([[self.fieldLength],[0.0],[-self.fieldHeight]])

        self.searchDimension_3D_start = self.startPosition
        self.searchDimension_3D_end = self.endPosition

        self.searchDimension_3D_projected_start = projectPosition_toPlane(pos_3D=self.searchDimension_3D_start,
                                                                          p_0=self.mapOrigin_3D,
                                                                          n_hat=n_hat)

        self.searchDimension_3D_projected_end = projectPosition_toPlane(pos_3D=self.searchDimension_3D_end,
                                                                          p_0=self.mapOrigin_3D,
                                                                          n_hat=n_hat)

        self.searchDimensions_2D_start = map_3D_to_2D(vec_3D=self.searchDimension_3D_projected_start,
                                                 n_hat=n_hat,
                                                 p0=self.mapOrigin_3D)
        
        self.searchdimensions_2D_end = map_3D_to_2D(vec_3D=self.searchDimension_3D_projected_end,
                                               n_hat=n_hat,
                                               p0=self.mapOrigin_3D)
        
        potato = 0



class PlanarVTOLSimplifiedParams:

    def __init__(self,
                 plane: MsgPlane,
                 fieldLength: float = 1000.0,
                 obstacleWidth: float = 100.0,
                 obstacleDepth: float = 50.0,
                 numObstacles_line: int = 3):
        
        #save the plane message
        self.plane = plane

        stepSize = fieldLength / numObstacles_line

        startStepSize = stepSize / 2.0
        
        north_start = startStepSize
        altitude_start = startStepSize

        #creates the dimensions of the individual obstacles (from the center as reference)
        #needs to be in the order of first depth, and then the variable dimensions 
        # (because this is in the order of )
        self.dimensions = np.array([[obstacleDepth],
                                    [obstacleWidth],
                                    [obstacleWidth]])
        
        #gets the search dimensions
        self.startPosition_2D = np.array([[0.0],[0.0]])
        self.endPosition_2D = np.array([[fieldLength],[fieldLength]])

        self.startPosition_3D = map_2D_to_3D_planeMsg(vec_2D=self.startPosition_2D,
                                                      plane_msg=self.plane)
        self.endPosition_3D = map_2D_to_3D_planeMsg(vec_2D=self.endPosition_2D,
                                                    plane_msg=self.plane)

        self.positionsList_2D = []
        #iterates over north
        for i in range(numObstacles_line):
            #iterates over altitude
            for j in range(numObstacles_line):
                currentNorth = north_start + i*stepSize
                currentAltitude = altitude_start + j*stepSize

                self.positionsList_2D.append(np.array([[currentNorth],[currentAltitude]]))


                

                

class MsgWorldMap:

    #creates the init function
    def __init__(self,
                 obstacleFieldType: MapTypes,
                 numDimensions_algorithm: int,
                 planarVTOL_Params: PlanarVTOLParams = None,
                 planarVTOLSimplified_Params: PlanarVTOLSimplifiedParams = None):
        

        #saves all fo the above
        self.obstacleFieldType = obstacleFieldType
        self.numDimensions_algorithm = numDimensions_algorithm

        #saves the parameters for each type of obstacle field
        self.pln_VTOL_Param = planarVTOL_Params

        self.planarVTOLSimplified_params = planarVTOLSimplified_Params

        if obstacleFieldType == MapTypes.PLANAR_VTOL:

            self.initPlanarVTOL_map()

            #sets the search dimensions
            self.searchDimensions_start = planarVTOL_Params.searchDimensions_2D_start
            self.searchDimensions_end = planarVTOL_Params.searchdimensions_2D_end


        elif obstacleFieldType == MapTypes.PLANAR_VTOL_SIMPLIFIED:

            self.initPlanarVTOLSimplified_map()

            self.searchDimensions_start = planarVTOLSimplified_Params.startPosition_2D
            self.searchDimensions_end = planarVTOLSimplified_Params.endPosition_2D

        self.generateAbMatricesLists()

        self.generateFeasibilityList()

        potato = 0

    
    def initPlanarVTOL_map(self):

        #creates map of randomized obstacles for the 
        #in the 2D planar space, we get the bounds for x and y
        x_min = self.pln_VTOL_Param.mapOrigin_2D.item(0)
        z_min = self.pln_VTOL_Param.mapOrigin_2D.item(1)

        x_max = x_min + self.pln_VTOL_Param.fieldLength
        z_max = z_min + self.pln_VTOL_Param.fieldHeight

        #gets the Q basis matrix
        self.Q = getPlaneBasis(n_hat=self.pln_VTOL_Param.n_hat)

        self.p0 = self.pln_VTOL_Param.mapOrigin_3D

        #gets the Rotation matrix from the unit vector
        Rot_subspaceToWorld = getRotFromUnitVec(unitVec=self.pln_VTOL_Param.n_hat)

        #gets the rotation matrix from the world to the subspace rotation
        Rot_worldToSubspace = Rot_subspaceToWorld.T

        #gets the list of the obstacles
        self.obstaclesList = []


        #iterates over all of the obstacles in the section
        for _ in range(self.pln_VTOL_Param.numObstacles):
            

            #gets the randomized x and Z coordinates for the randomized position
            random_x = np.random.uniform(low=x_min, high=x_max)
            random_z = np.random.uniform(low=z_min, high=z_max)

            #sets the 2D position
            pos_random_2D = np.array([[random_x],[random_z]])

            #gets the random position in 3D world frame
            pos_random_3D_world = getWorldPos(pos_2D=pos_random_2D,
                                              Q=self.Q,
                                              p0=self.p0)

            #now we get a randomized shape for the obstacle
            random_length = np.random.uniform(low=self.pln_VTOL_Param.obstacleMinWidth,
                                              high=self.pln_VTOL_Param.obstacleMaxWidth)
            random_height  = np.random.uniform(low=self.pln_VTOL_Param.obstacleMinWidth,
                                              high=self.pln_VTOL_Param.obstacleMaxWidth)
            
            dimensions = np.array([[self.pln_VTOL_Param.obstacleDepth],[random_length],[random_height]])
            
            #gets the random position in the Planar frame
            pos_random_3D_subspace = Rot_worldToSubspace @ pos_random_3D_world

            #creates the rectangular object
            tempObstacle = RectangularObstacle(dimensions_obs=dimensions,
                                                  translation_obs=pos_random_3D_subspace,
                                                  rotation_obsToWorld=Rot_subspaceToWorld)

            #appends to the obstacle list
            self.obstaclesList.append(tempObstacle)

            #gets the temp SFC from the obstacle
            tempSFC = tempObstacle.getSFC()

            tomato = 0            

        potato = 0


    def initPlanarVTOLSimplified_map(self):

        plane_msg = self.planarVTOLSimplified_params.plane

        self.Q = getPlaneBasis(n_hat=plane_msg.n_hat)

        self.p0 = plane_msg.origin_3D

        #gets the rotation from the subspace to the world frame
        Rot_subspaceToWorld = getRotFromUnitVec(unitVec=plane_msg.n_hat)
        Rot_worldToSubspace = Rot_subspaceToWorld.T
        
        self.obstaclesList = []

        #iterates over all of the positions in the position list
        for position_2D in self.planarVTOLSimplified_params.positionsList_2D:

            #gets the position in 3 Dimensions in the world frame
            position_3D_world = map_2D_to_3D_planeMsg(vec_2D=position_2D,
                                                plane_msg=plane_msg)
            
            #gets the position in the subspace frame
            position_3D_subspace = Rot_worldToSubspace @ position_3D_world

            #creates the Rectangular obstacle in this orientation
            tempObstacle = RectangularObstacle(dimensions_obs=self.planarVTOLSimplified_params.dimensions,
                                               translation_obs=position_3D_subspace,
                                               rotation_obsToWorld=Rot_subspaceToWorld)
            self.obstaclesList.append(tempObstacle)



    #gets all of the A and b matrices for each of the obstacles as a large list
    def generateAbMatricesLists(self):

        self.Ab_3D_list = []

        self.Ab_2D_list = []



        for obstacle in self.obstaclesList:

            obstacleSFCTemp = obstacle.getSFC()

            A_3D_temp, b_3D_temp = obstacleSFCTemp.getAbMatrices()
            self.Ab_3D_list.append([A_3D_temp, b_3D_temp])

            #case this is a 2D map
            if self.numDimensions_algorithm == 2:
                #gets the 2D equivalents
                A_2D = A_3D_temp @ self.Q
                b_2D = b_3D_temp - A_3D_temp @ self.p0
                self.Ab_2D_list.append([A_2D, b_2D])
                
    def get_obstacles(self):
        return self.obstaclesList
    
    #returns the 3D Ab list
    def get_Ab_3D(self):
        return self.Ab_3D_list
    
    #returns the 2D Ab list
    def get_Ab_2D(self):
        return self.Ab_2D_list
    
    #returns the Q matrix
    def get_Q(self):
        return self.Q
    

    def generateFeasibilityList(self):

        self.feasibilityList = []
        for Ab_pair in self.Ab_2D_list:

            A = Ab_pair[0]
            b = Ab_pair[1]

            dummyVariable = np.ones(A.shape[1])

            #checks the feasibility of this pair
            result = linprog(dummyVariable, A_ub=A, b_ub=b, method='highs')

            self.feasibilityList.append(result.success)

#defines the helper function to calculate the 3D world position given a position
#in a 2D plane, assuming that the p0 used to define the plane is the origin
#arguments:
#pos_2D: the position in the 2D space of the plane
#Q: the orthonormal basis for the plane perpendicular to the normal. It is also the nullspace
    #of the n_hat vector. 3x2 matrix
#p0: the position of the plane's origin expressed in the world 3D frame
def getWorldPos(pos_2D: np.ndarray,
                Q: np.ndarray,
                p0: np.ndarray):
    
    #sets the 3D position
    pos_3D = p0 + Q @ pos_2D
    return pos_3D




