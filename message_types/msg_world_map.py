import numpy as np
import parameters.planner_parameters as PLAN
import parameters.planarVTOL_map_parameters as PLANAR_PARAM
from tools.obstacles import RectangularObstacle
import time
from enum import Enum
from tools.getEulerUnit import getRotFromUnitVec


#creates the enumeration for 

class MapTypes(str, Enum):
    CITY = 'City'
    FLOATING_BLOCKS = 'FloatingBlocks'
    PLANAR_VTOL = 'PlanarVTOL'
    MAZE = 'Maze'


class PlanarVTOLParams:

    def __init__(self,
                 fieldLength: float = PLANAR_PARAM.mapLength,
                 fieldHeight: float = PLANAR_PARAM.mapHeight,
                 obstacleMaxWidth: float = PLANAR_PARAM.obstacleMaxWidth,
                 obstacleMinWidth: float = PLANAR_PARAM.obstacleMinWidth,
                 obstacleDepth: float = PLANAR_PARAM.obstacleDepth,
                 obstacleOrigin_2D: np.ndarray = None,
                 obstacleOrigin_3D: np.ndarray = None,
                 n_hat: np.ndarray = None,
                 numObstacles: int = PLANAR_PARAM.numObstacles):
        
        self.fieldLength = fieldLength
        self.fieldHeight = fieldHeight
        self.obstacleMaxWidth = obstacleMaxWidth
        self.obstacleMinWidth = obstacleMinWidth
        self.obstacleDepth = obstacleDepth
        self.obstacleOrigin_2D = obstacleOrigin_2D
        self.obstacleOrigin_3D = obstacleOrigin_3D
        self.n_hat = n_hat
        self.numObstacles = numObstacles


class MsgWorldMap:

    #creates the init function
    def __init__(self,
                 obstacleFieldType: MapTypes,
                 numDimensions_algorithm: int,
                 planarVTOL_Params: PlanarVTOLParams = None):
        

        #saves all fo the above
        self.obstacleFieldType = obstacleFieldType
        self.numDimensions_algorithm = numDimensions_algorithm

        #saves the parameters for each type of obstacle field
        self.pln_VTOL_Param = planarVTOL_Params

        if obstacleFieldType == MapTypes.PLANAR_VTOL:

            self.initPlanarVTOL_map()


        self.generateAbMatricesLists()

        potato = 0

    
    def initPlanarVTOL_map(self):

        #creates map of randomized obstacles for the 
        #in the 2D planar space, we get the bounds for x and y
        x_min = self.pln_VTOL_Param.obstacleOrigin_2D.item(0)
        z_min = self.pln_VTOL_Param.obstacleOrigin_2D.item(1)

        x_max = x_min + self.pln_VTOL_Param.fieldLength
        z_max = z_min + self.pln_VTOL_Param.fieldHeight

        #gets the Q basis matrix
        Q = getPlaneBasis(n_hat=self.pln_VTOL_Param.n_hat)

        p0 = self.pln_VTOL_Param.obstacleOrigin_3D

        #gets the Rotation matrix from the unit vector
        Rot_subspaceToWorld = getRotFromUnitVec(unitVec=self.pln_VTOL_Param.n_hat)

        #gets the rotation matrix from the world to the subspace rotation
        Rot_worldToSubspace = Rot_subspaceToWorld.T

        #gets the list of the obstacles
        self.obstaclesList = []

        #iterates over all of the obstacles in the section
        for i in range(self.pln_VTOL_Param.numObstacles):
            

            #gets the randomized x and Z coordinates for the randomized position
            random_x = np.random.uniform(low=x_min, high=x_max)
            random_z = np.random.uniform(low=z_min, high=z_max)

            #sets the 2D position
            pos_random_2D = np.array([[random_x],[random_z]])

            #gets the random position in 3D world frame
            pos_random_3D_world = getWorldPos(pos_2D=pos_random_2D,
                                              Q=Q,
                                              p0=p0)

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

        potato = 0

    #gets all of the A and b matrices for each of the obstacles as a large list
    def generateAbMatricesLists(self):



        self.Ab_list = []

        for obstacle in self.obstaclesList:

            obstacleSFCTemp = obstacle.getSFC()

            A_temp, b_temp = obstacleSFCTemp.getAbMatrices()
            self.Ab_list.append([A_temp, b_temp])
                


    def get_obstacles(self):
        return self.obstaclesList

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


def getPlaneBasis(n_hat: np.ndarray):


    #gets the SVD of the n hat
    U, S, Vt = np.linalg.svd(n_hat.T)

    #gets the basis from the Vt
    Q = Vt[:,1:]


    return Q