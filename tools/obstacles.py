#creates the class to create obstacles
import numpy as np
import parameters.planner_parameters as PLAN


class RectangularObstacle:

    #dimensions is the size of each dimension (length width height) in the Obstacle Frame of reference
    #translation is the translation from the origin to the center of the Obstacle in the Obstacle's frame of reference
    #rotation from the Obstacle Frame of reference to the world frame of reference
    def __init__(self,
                 dimensions_obs: np.ndarray,
                 translation_obs: np.ndarray,
                 rotation_obsToWorld: np.ndarray):

        self.dimensions_obs = dimensions_obs
        self.translation_obs = translation_obs
        self.rotation_obsToWorld = rotation_obsToWorld


        self.numDimensions = np.size(self.dimensions_obs)

        if self.numDimensions == 2:
            self.init_2D()
        elif self.numDimensions == 3:
            self.init_3D()
        else:
            raise ValueError("invalid number of dimensions")
    



    #creates the init function for the size 2 maps
    def init_2D(self):

        #gets the min and max bounds
        minBounds, maxBounds = self.getBounds_obstacleFrame()

        #gets the mins and maxes for x and y
        x_min = minBounds.item(0)
        y_min = minBounds.item(1)

        x_max = maxBounds.item(0)
        y_max = maxBounds.item(1)

        #gets the unrotated points
        vertices_objectFrame = [np.array([[x_min],[y_min]]),
                                np.array([[x_max],[y_min]]),
                                np.array([[x_max],[y_max]]),
                                np.array([[x_min],[y_max]])]
        
        #gets the vertices in the world frame
        self.vertices_worldFrame = self.rotation_obsToWorld @ vertices_objectFrame

        pass

    def init_3D(self):

        pass

    #################################################
    #2D section
    #degines the function to get the vertices for the 
    def getVertices_2D(self):

        pass
    

    ######################################################
    #3D section

    def getBounds_obstacleFrame(self):

        #gets the min mound
        minBounds = self.translation_obs - self.dimensions_obs/2.0
        #and the maximum bounds
        maxBounds = self.translation_obs + self.dimensions_obs/2.0
        #returns the min and max bounds in that order
        return minBounds, maxBounds
