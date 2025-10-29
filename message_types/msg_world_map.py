import numpy as np
import parameters.planner_parameters as PLAN
import parameters.planarVTOL_map_parameters as PLANAR_PARAM
from tools.obstacles import RectangularObstacle
import time
from enum import Enum


#creates the enumeration for 

class MapTypes(str, Enum):
    CITY = 'City'
    FLOATING_BLOCKS = 'FloatingBlocks'
    PLANAR_VTOL = 'PlanarVTOL'
    MAZE = 'Maze'


class PlanarVTOLParams:

    def __init__(self,
                 fieldLength: float,
                 fieldHeight: float,
                 obstacleMaxWidth: float,
                 obstacleMinWidth: float,
                 obstacleDepth: float,
                 obstacleOrigin_2D: np.ndarray,
                 obstacleOrigin_3D: np.ndarray,
                 n_hat: np.ndarray,
                 numObstacles: int):
        
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

    
    def initPlanarVTOL_map(self):

        #creates map of randomized obstacles for the 
        #in the 2D planar space, we get the bounds for x and y
        x_min = self.pln_VTOL_Param.obstacleOrigin_2D.item(0)
        z_min = self.pln_VTOL_Param.obstacleOrigin_2D.item(1)

        x_max = x_min + self.pln_VTOL_Param.fieldLength
        z_max = z_min + self.pln_VTOL_Param.fieldHeight


        #iterates over all of the obstacles in the section
        for i in range(self.pln_VTOL_Param.numObstacles):
            

            #gets the randomized x and Z coordinates for the randomized position
            random_x = np.random.uniform(low=x_min, high=x_max)
            random_z = np.random.uniform(low=z_min, high=z_max)

            #sets the 2D position
            pos_random_2D = np.array([[random_x],[random_z]])

            #gets the Q matrix for the initial definition

            #gets the random position in 3D world frame
            pos_random_3D_world = getWorldPos(pos_2D=pos_random_2D,
                                              Q=)

            #now we get a randomized shape for the obstacle
            random_length = np.random.uniform(low=self.pln_VTOL_Param.obstacleMinWidth,
                                              high=self.pln_VTOL_Param.obstacleMaxWidth)
            random_height  = np.random.uniform(low=self.pln_VTOL_Param.obstacleMinWidth,
                                              high=self.pln_VTOL_Param.obstacleMaxWidth)




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
    U, S, Vt = np.linalg.svd(n_hat)

    #gets the basis from the Vt
    Q = Vt[:,1:]


    return Q