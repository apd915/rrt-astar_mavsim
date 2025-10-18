import numpy as np
import parameters.flightCorridor_parameters as FLPARAM
from shapely.geometry import MultiPoint

from eVTOL_BSplines.submodules.path_generator.path_generation.safe_flight_corridor import SFC, SFC_Data
from tools.rotations import euler_to_rotation


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


    #gets the sfc
    def getSFC(self):
        
        #calls the the function to get the respective sfc
        if self.numDimensions == 2:
            #
            tempSFC = self.__getSFC_2d()
        elif self.numDimensions == 3:
            tempSFC = self.__getSFC_3d()

        #now, we return the temp SFC
        return tempSFC

    def __getSFC_2d(self):
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

        #grom the center vector norm, we get the north and east components
        centerVectorNorm_north = centerVectorNorm.item(0)
        centerVectorNorm_east = centerVectorNorm.item(1)

        #gets the yaw angle
        self.yaw_angle = np.arctan2(centerVectorNorm_east, centerVectorNorm_north)

        #gets the rotation matrix from the pitch, yaw, and roll angles
        self.R_SFCToWorld = euler_to_rotation(phi=0.0,
                                              theta=0.0,
                                              psi=self.yaw_angle)

        #gets the rotation for world to sfc
        self.R_WorldToSFC = self.R_SFCToWorld.T

        #gets the translation in the sfc grame
        self.translation_SFC = self.R_WorldToSFC @ self.translation_world

        tempSFC = SFC(dimensions=self.dimensions,
                      translation=self.translation_SFC,
                      rotation=self.R_SFCToWorld)
        
        return tempSFC

    def __getSFC_3d(self):

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