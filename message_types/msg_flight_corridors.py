import numpy as np
import parameters.flightCorridor_parameters as FLPARAM
from shapely.geometry import MultiPoint
from tools.safeFlightCorridor import SFC
from tools.rotations import euler_to_rotation, euler_to_rotation_2D


#creates the message class for the 3D flight corridor
class MsgFlightCorridor:

    def __init__(self,
                 primaryPosition: np.ndarray, #the start position of the SFC
                 secondaryPosition: np.ndarray, #the end position of the SFC
                 numDimensions: int = 2,#sets the number of dimensions, and whether we build 2D or 3D
                 primaryPosition_index: int = np.inf, #the index in the tree for the start position
                 secondaryPosition_index: int = np.inf, #the index in the tree for the end position
                 width: float = FLPARAM.width, #the width of the Flight corridor
                 height: float = FLPARAM.height, #the height of the flight corridor
                 startExtension: float = FLPARAM.startExtension,
                 endExtension: float = FLPARAM.endExtension):
        
        self.numDimensions = numDimensions

        #saves all of these things
        self.primaryPosition = primaryPosition
        self.secondaryPosition = secondaryPosition
        self.primaryPosition_index = primaryPosition_index
        self.secondaryPosition_index = secondaryPosition_index
        self.width = width
        self.height = height
        self.startExtension = startExtension
        self.endExtension = endExtension


        #case this is a 2D flight corridor, we want to extract the altitude, from the primary position
        if numDimensions == 2:
            self.altitude = primaryPosition.item(2)


        #generates the SFC for this
        self.sfc = self.__generateSFC()



    #gets the sfc
    def __generateSFC(self)->SFC:
        
        #calls the the function to get the respective sfc
        if self.numDimensions == 2:
            #
            tempSFC = self.__generateSFC_2d()
        elif self.numDimensions == 3:
            tempSFC = self.__generateSFC_3d()

        #now, we return the temp SFC
        return tempSFC
    
    def getAbMatrices(self):
        #gets the temp SFC
        tempSFC = self.__generateSFC()

        #gets the A and b
        A_temp, b_temp = tempSFC.generateAbMatrices()

        return A_temp, b_temp

    def __generateSFC_2d(self):
        #gets the center vector (vector from primary pivot position to secondary pivot Position)
        self.centerVector = self.secondaryPosition - self.primaryPosition

        #get the centerLength
        self.centerLength = np.linalg.norm(self.centerVector)

        #gets the center vector norm
        centerVectorNorm = self.centerVector / self.centerLength

        #gets the total length
        self.length = self.centerLength + self.startExtension + self.endExtension

        #sets the dimensions for the SFC
        self.dimensions = np.array([[self.length],
                                    [self.width]])

        #gets the centerPosition. the center position is the translation vector in the world frame
        centerPosition = self.primaryPosition + centerVectorNorm * (self.centerLength / 2.0)
        self.translation_world = centerPosition

        translation_world_2D = self.translation_world[:2,:]

        #grom the center vector norm, we get the north and east components
        centerVectorNorm_north = centerVectorNorm.item(0)
        centerVectorNorm_east = centerVectorNorm.item(1)

        #gets the yaw angle
        self.yaw_angle = np.arctan2(centerVectorNorm_east, centerVectorNorm_north)
        
        self.R_SFCToWorld_2D = euler_to_rotation_2D(psi=self.yaw_angle)

        self.R_worldToSFC_2D = self.R_SFCToWorld_2D.T


        #gets the translation in the sfc grame
        self.translation_SFC = self.R_worldToSFC_2D @ translation_world_2D

        tempSFC = SFC(dimensions=self.dimensions,
                      translation=self.translation_SFC,
                      rotation=self.R_SFCToWorld_2D)
        
        return tempSFC

    def __generateSFC_3d(self):

        #gets the center vector (vector from primary pivot position to secondary pivot Position)
        self.centerVector = self.secondaryPosition - self.primaryPosition

        #get the centerLength
        self.centerLength = np.linalg.norm(self.centerVector)

        #gets the center vector norm
        centerVectorNorm = self.centerVector / self.centerLength

        #gets the total length
        self.length = self.centerLength + self.startExtension + self.endExtension


        #sets the dimensions for the SFC
        self.dimensions = np.array([[self.length],
                                    [self.width],
                                    [self.height]])
        
        #gets the centerPosition. the center position is the translation vector in the world frame
        centerPosition = self.primaryPosition + centerVectorNorm * (self.centerLength / 2.0)
        self.translation_world = centerPosition

        #grom the center vector norm, we get the north and east components
        centerVectorNorm_north = centerVectorNorm.item(0)
        centerVectorNorm_east = centerVectorNorm.item(1)
        centerVectorNorm_down = centerVectorNorm.item(2)

        #gets the yaw angle
        self.yaw_angle = np.arctan2(centerVectorNorm_east, centerVectorNorm_north)

        #creates the center vector norm projection onto the NE plane
        centerVectorNorm_NEProjection = np.array([[centerVectorNorm_north],
                                                  [centerVectorNorm_east],
                                                  [0.0]])
        
        #gets the length of the center vector norm
        centerVectorNorm_NEProjection_length = np.linalg.norm(centerVectorNorm_NEProjection)
        
        #TODO confirm that this is the correct sign for pitch
        #gets the pitch angle from the length of the projection vector and the down length
        self.pitch_angle = -np.arctan2(centerVectorNorm_down, centerVectorNorm_NEProjection_length)

        #gets the rotation matrix from the pitch, yaw, and roll angles
        self.R_SFCToWorld = euler_to_rotation(phi=0.0,
                                         theta=self.pitch_angle,
                                         psi=self.yaw_angle)
        
        #gets the rotation for world to sfc
        self.R_WorldToSFC = self.R_SFCToWorld.T

        #gets the translation in the sfc grame
        self.translation_SFC = self.R_WorldToSFC @ self.translation_world

        #now, with these things, we can create the safe flight corridor

        tempSFC = SFC(dimensions=self.dimensions,
                      translation=self.translation_SFC,
                      rotation=self.R_SFCToWorld)
        
        return tempSFC
    
    def getNumDimensions(self):
        return self.numDimensions
    
    #gets the already pregenerated sfc
    def getSFC(self)->SFC:
        return self.sfc
    


    #from the safe Flight Corridor Message, we get the vertices of the SFC, taking into account the altitude
    def getVertices(self):

        if self.numDimensions == 2:

            return self.__getVertices_2D()
        
        elif self.numDimensions == 3:

            return self.__getVertices_3D()

    

    def __getVertices_2D(self):

        #gets the vertices from the sfc
        sfc_normals, sfc_vertices = self.sfc.getNormalsVertices()

        vertices_out = []

        for vertex_temp in sfc_vertices:

            vertex_out_temp = np.concatenate((vertex_temp, np.array([[self.altitude]])), axis=0)

            #appends to the vertices out
            vertices_out.append(vertex_out_temp)

        return vertices_out

    def __getVertices_3D(self):

        pass
