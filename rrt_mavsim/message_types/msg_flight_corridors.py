import numpy as np
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PARAM
from rrt_mavsim.tools.safeFlightCorridor import SFC
from rrt_mavsim.tools.rotations import euler_to_rotation, euler_to_rotation_2D

#A note on the following:
#if this is a 2D flight corridor, the positions will be given in 2D,
#that is, in the frame of the work plane.
#I attempt to distinguish between both


class MsgFlightCorridor:


    def __init__(self,
                 numDimensions: int,
                 primaryPosition: np.ndarray,
                 secondaryPosition: np.ndarray,
                 primaryPosition_index: int = np.inf,
                 secondaryPosition_index: int = np.inf,
                 width: float = FLIGHT_PARAM.width,
                 height: float = FLIGHT_PARAM.height,
                 startExtension_length: float = FLIGHT_PARAM.startExtension,
                 endExtension_length: float = FLIGHT_PARAM.endExtension):
        
        self.numDimensions = numDimensions

        #saves all of these things
        self.primaryPosition = primaryPosition
        self.secondaryPosition = secondaryPosition
        self.primaryPosition_index = primaryPosition_index
        self.secondaryPosition_index = secondaryPosition_index
        self.width = width
        self.height = height
        self.startExtension_length = startExtension_length
        self.endExtension_length = endExtension_length

        self.__generateSFC()


    def __generateSFC(self):

        if self.numDimensions == 2:

            self.sfc = self.__generateSFC_2D()
        elif self.numDimensions == 3:

            self.sfc = self.__generateSFC_3D()

    
    #the 2D version of the SFC generation
    def __generateSFC_2D(self):
        #gets the center vector (vector from primary pivot position to secondary pivot Position)
        self.centerVector = self.secondaryPosition - self.primaryPosition

        #get the centerLength
        self.centerLength = np.linalg.norm(self.centerVector)

        #gets the center vector norm
        centerVectorNorm = self.centerVector / self.centerLength

        #gets the total length
        self.length = self.centerLength + self.startExtension_length + self.endExtension_length

        #sets the dimensions for the SFC
        self.dimensions = np.array([[self.length],
                                    [self.width]])

        #gets the centerPosition. the center position is the translation vector in the world frame
        centerPosition = self.primaryPosition + centerVectorNorm * (self.centerLength / 2.0)
        self.translation_plane = centerPosition


        #grom the center vector norm, we get the north and east components
        centerVectorNorm_north = centerVectorNorm.item(0)
        centerVectorNorm_east = centerVectorNorm.item(1)

        #gets the yaw angle
        self.yaw_angle = np.arctan2(centerVectorNorm_east, centerVectorNorm_north)

        #gets the Rotation from the SFC frame to the plane frame 
        # (that is this is all in the to R^2 space defined by u1 and u2)
        self.R_SFCToPlane_2D = euler_to_rotation_2D(psi=self.yaw_angle)

        #gets its inverse
        self.R_PlaneToSFC_2D = self.R_SFCToPlane_2D.T

        #gets  the translation in the sfc frame (all in the 2D plane's R2 frame)
        self.translation_SFC = self.R_PlaneToSFC_2D @ self.translation_plane


        #creates the sfc for this
        tempSFC = SFC(dimensions=self.dimensions,
                      translation=self.translation_SFC,
                      rotation=self.R_SFCToPlane_2D)
        
        return tempSFC
    

    #the 3D version of the SFC generation
    def __generateSFC_3D(self):

        #gets the center vector (vector from primary pivot position to secondary pivot Position)
        self.centerVector = self.secondaryPosition - self.primaryPosition

        #get the centerLength
        self.centerLength = np.linalg.norm(self.centerVector)

        #gets the center vector norm
        centerVectorNorm = self.centerVector / self.centerLength

        #gets the total length
        self.length = self.centerLength + self.startExtension_length + self.endExtension_length


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

    def getSFC(self):
        return self.sfc

    def getAbMatrices(self):
        A, b = self.sfc.getAbMatrices()
        return A, b