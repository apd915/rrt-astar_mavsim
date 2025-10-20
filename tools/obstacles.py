#creates the class to create obstacles
import numpy as np
import parameters.planner_parameters as PLAN



#NOTE: I am doing this all in 3D space. The problems with my code inevitably arise when I'm being inconsistent or having
#to translate between 2D and 3D vectors. So, when I am creating a 2D object, that is like in a plane in 3D space
#with some specified altitude.


class RectangularObstacle:

    #dimensions is the size of each dimension (length width height) in the Obstacle Frame of reference
    #translation is the translation from the origin to the center of the Obstacle in the Obstacle's frame of reference
    #rotation from the Obstacle Frame of reference to the world frame of reference
    def __init__(self,
                 dimensions_obs: np.ndarray,
                 translation_obs: np.ndarray,
                 rotation_obsToWorld: np.ndarray,
                 building_height: float = None):

        self.dimensions_obs = dimensions_obs
        self.translation_obs = translation_obs
        self.rotation_obsToWorld = rotation_obsToWorld

        self.building_height = building_height

        #that is, the number of dimensions specified in the obstacle input
        self.numDimensions = np.size(self.dimensions_obs)

        if self.numDimensions == 2:
            self.init_2D()
        elif self.numDimensions == 3:
            self.init_3D()
        else:
            raise ValueError("invalid number of dimensions")
    
        tomato = 0


    #creates the init function for the size 2 maps
    def init_2D(self):

        #gets the min and max bounds
        minBounds, maxBounds = self.getBounds_obstacleFrame()

        #gets the mins and maxes for x and y
        x_min = minBounds.item(0)
        y_min = minBounds.item(1)

        x_max = maxBounds.item(0)
        y_max = maxBounds.item(1)

        #for the following vertices, I am referring to the coordinates of the projection of the building
        #onto a 2D plane. Then the building will have both top and bottom vertices (Same but just shifted by the altitude)

        #gets the unrotated points.
        vertices_unshifted_objectFrame = np.array([[x_min, x_max, x_max, x_min],
                                                   [y_min, y_min, y_max, y_max]])
        
        #gets the shifted vertices in the object frame
        vertices_shifted_objectFrame = vertices_unshifted_objectFrame + self.translation_obs
        
        #gets the vertices in the world frame for the planar vertices (on a projected subspace)
        vertices_worldFrame_2D_plane = self.rotation_obsToWorld @ vertices_shifted_objectFrame

        potato = 0

        

        pass

    def init_3D(self):

        pass

    #################################################
    #2D section
    #degines the function to get the vertices for the 
    def getVertices_2D(self):

        return self.vertices_worldFrame_2D
    

    ######################################################
    #3D section

    def getBounds_obstacleFrame(self):

        #gets the min mound
        minBounds = self.translation_obs - self.dimensions_obs/2.0
        #and the maximum bounds
        maxBounds = self.translation_obs + self.dimensions_obs/2.0
        #returns the min and max bounds in that order
        return minBounds, maxBounds
