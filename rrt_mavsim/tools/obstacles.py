#creates the class to create obstacles
import numpy as np
import rrt_mavsim.parameters.planner_parameters as PLAN
from rrt_mavsim.message_types.msg_safeFlightCorridor import Msg_SFC
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import *
from eVTOL_BSplines.submodules.path_generator.path_generation.obstacle import Obstacle



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


        #gets the translation in the world frame
        self.translation_world = self.rotation_obsToWorld @ self.translation_obs

        self.building_height = building_height

        self.SFC = Msg_SFC(dimensions=self.dimensions_obs,
                       translation=self.translation_obs,
                       rotation=self.rotation_obsToWorld)

        #that is, the number of dimensions specified in the obstacle input
        self.numDimensions = np.size(self.dimensions_obs)

        if self.numDimensions == 2:
            self.init_2D()
        elif self.numDimensions == 3:
            self.init_3D()
        else:
            raise ValueError("invalid number of dimensions")
    



    #creates the init function for the size 2 maps
    def init_2D(self):

        if self.building_height is None:
            raise ValueError("No building Height Selected")

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
        self.vertices_projection_2D_plane = self.rotation_obsToWorld @ vertices_shifted_objectFrame

        #section to get the Normal vectors



        ###################################################################
        #section for constructing the 3D building
        numVerticesFace = np.shape(vertices_unshifted_objectFrame)[1]

        #creates the altitude vectors
        altitudeVector_bottom = np.full((1,numVerticesFace), 0)
        altitudeVector_top = np.full((1,numVerticesFace), self.building_height)
        
        bottomVectors = np.concatenate((self.vertices_projection_2D_plane, altitudeVector_bottom), axis=0)
        topVectors = np.concatenate((self.vertices_projection_2D_plane, altitudeVector_top), axis=0)

        self.vertices_building_worldFrame = np.concatenate((bottomVectors, topVectors), axis=1)

        potato = 0

        

        pass

    def init_3D(self):

        #gets the bounds for the 3 dimensions
        minBounds, maxBounds = self.getBounds_obstacleFrame()

        x_min = minBounds.item(0)
        y_min = minBounds.item(1)
        z_min = minBounds.item(2)

        x_max = maxBounds.item(0)
        y_max = maxBounds.item(1)
        z_max = maxBounds.item(2)

        #creates the complete vector of unrotated vertices
        vertices_unshifted_obsFrame = np.array([[x_min, x_max, x_max, x_min, x_min, x_max, x_max, x_min],
                                                [y_min, y_min, y_max, y_max, y_min, y_min, y_max, y_max],
                                                [z_min, z_min, z_min, z_min, z_max, z_max, z_max, z_max]])
        #gets them shifted in the obstacle frame
        vertices_shifted_obsFrame = vertices_unshifted_obsFrame + self.translation_obs

        #rotates them to the world frame
        self.vertices_shifted_worldFrame_3D = self.rotation_obsToWorld @ vertices_shifted_obsFrame

        pass

    #################################################
    #2D section
    #degines the function to get the vertices for the 
    def getVertices_projection_2D(self):

        return self.vertices_projection_2D_plane
    
    def getVertices_building_2D(self):

        return self.vertices_building_worldFrame
    
    def getVertices_obstacle_3D(self):
        return self.vertices_shifted_worldFrame_3D
    

    def getVertices_projection_2D_list(self):

        verticesShape = np.shape(self.vertices_projection_2D_plane)
        numVertices = verticesShape[1]

        vertices_list = []
        #
        for i in range(numVertices):

            tempVertex = (self.vertices_projection_2D_plane)[:,i:(i+1)]
            vertices_list.append(tempVertex)

        return vertices_list
    
    def getVertices_building_2D_list(self):

        verticesShape = np.shape(self.vertices_building_worldFrame)
        numVertices = verticesShape[1]

        vertices_list = []

        for i in range(numVertices):

            tempVertex = (self.vertices_building_worldFrame)[:,i:(i+1)]
            vertices_list.append(tempVertex)

        return vertices_list
    
    def getVertices_obstacle_3D_list(self):

        verticesShape = np.shape(self.vertices_shifted_worldFrame_3D)
        numVertices = verticesShape[1]

        verticesList = []

        for i in range(numVertices):

            tempVertex = self.vertices_shifted_worldFrame_3D[:,i:(i+1)]
            verticesList.append(tempVertex)
        return verticesList


    ######################################################
    #3D section

    def getBounds_obstacleFrame(self):

        #gets the min mound
        minBounds = -self.dimensions_obs/2.0
        #and the maximum bounds
        maxBounds = self.dimensions_obs/2.0
        #returns the min and max bounds in that order
        return minBounds, maxBounds
    


    def getSFC(self):
        return self.SFC
    

    #defines the function to get the translation in the world frame
    def getTranslationWorld(self):
        return self.translation_world
    



    #defines the function to get the inscribed Cylinder
    #Arguments: 
    #1. dimensionDiameter_index - the index of the dimension from the rectangular obstacle
    #      which will become the diameter of the cylinder. (the circle will touch the edges of this dimension)
    #2. dimensionAxis_index - the index the axis of the cylinder will be on
    def getInscribedCylinder(self,
                             dimensionDiameter_index: int,
                             dimensionAxis_index: int):
        


        pass

    #defines the function to get the circular obstacle object from the rectangular obstacle
    #arguments:
    #dimensionDiameter_index: which dimension index will be used to set the diameter of the circle
    #dimensionAxis_index: which dimension index will be used to set the axis it will be aligned to
    def getCircularObstacle(self,
                            plane_msg: MsgPlane):
        
        #gets the center position of the circular obstacle in the world frame
        translation_world_3D = self.getTranslationWorld()

        #gets the projected position onto the plane, but still in 3 dimensions
        translation_world_3D_projected = projectPositionToPlane_planeMsg(pos_3D=translation_world_3D,
                                                                         plane_msg=plane_msg)
        
        #gets the translation in the world 2D projected
        translation_world_2D = map_3D_to_2D_planeMsg(vec_3D=translation_world_3D_projected,
                                                     plane_msg=plane_msg)
        

        #gets the two applicable dimensions
        n_size = self.dimensions_obs.item(1)
        a_size = self.dimensions_obs.item(2)

        #gets the smaller value
        minimumDimension = min(n_size, a_size)

        #gets the radius
        radius = minimumDimension/2.0

        #now, with the information we create the circular obstacle
        circularObstacle = Obstacle(center=translation_world_2D,
                                    radius=radius)

        return circularObstacle
        




class CylindricalObstacle: 

    #dimensisons 
    def __init__(self,
                 radius: float,
                 objectHeight: float,
                 translation_obsFrame: np.ndarray,
                 rotation_obsToWorld: np.ndarray):
        

        self.radius = radius
        self.objectHeight = objectHeight
        self.translation_obsFrame = translation_obsFrame
        self.rotation_obsToWorld = rotation_obsToWorld

        #the translation in the world frame
        self.translation_worldFrame = self.rotation_obsToWorld @ self.translation_obsFrame


        