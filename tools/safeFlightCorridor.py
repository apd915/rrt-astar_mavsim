import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
from shapely.geometry import MultiPoint


#defines the new list of edge vertices indices
edges_verticesIndices_3D = [[0,1], #pair 0 
                            [0,3], #pair 1 
                            [0,4], #pair 2
                            [1,2], #pair 3
                            [1,5], #pair 4
                            [2,3], #pair 5
                            [2,6], #pair 6
                            [3,7], #pair 7
                            [4,5], #pair 8
                            [4,7], #pair 9
                            [5,6], #pair 10
                            [6,7]] #pair 11

class SFC:

    def __init__(self,
                 dimensions: np.ndarray,
                 translation: np.ndarray,
                 rotation: np.ndarray):
        
        self.dimensions = dimensions
        self.translation = translation
        self.rotation = rotation

        self.numDimensions = np.size(self.dimensions)


    def getRotatedBounds(self):
        max_bounds = self.translation + self.dimensions/2.0
        min_bounds = self.translation - self.dimensions/2.0

        return min_bounds, max_bounds


    #function to get the A and b matrices
    def getAbMatrices(self):

        normals, vertices = self.getNormalsVertices()

        numDimensions = np.size(self.dimensions)

        self.A = np.ndarray((0, numDimensions))

        self.b = np.ndarray((0, 1))

        for normal, vertex in zip(normals, vertices):

            self.A = np.concatenate((self.A, normal.T), axis=0)
            
            b_value = normal.T @ vertex
            self.b = np.concatenate((self.b, b_value), axis=0)

        return self.A, self.b

    def getNormalsVertices(self):

        if self.numDimensions == 2:

            normalsList, verticesList = self.__getNormalsVertices_2d()

        elif self.numDimensions == 3:
            normalsList, verticesList = self.__getNormalsVertices_3d()

        return normalsList, verticesList


    #gets the normal vectors with each corresponding vertex.
    #IMPORTANT NUANCE: Normal vectors are pointing OUTWARD from the convex hull
    #this makes it easier to use the less than or equal operator than 
    def __getNormalsVertices_2d(self):
        
        #gets the x dimension
        x_dimension = self.dimensions[0,0]
        #gets the y dimension
        y_dimension = self.dimensions[1,0]

        x_min = -x_dimension/2
        x_max = x_dimension/2

        y_min = -y_dimension/2
        y_max = y_dimension/2

        #creates the initial vertices before rotation and translation
        initialVertices = np.array([[x_max, x_max, x_min, x_min],
                                    [y_max, y_min, y_min, y_max]])
        


        #gets the two rotation matrices
        rotation_CL_to_world = self.rotation
        rotation_world_to_CL = rotation_CL_to_world.T

        #gets the rotation of the initial vertices
        rotatedVertices = rotation_CL_to_world @ initialVertices

        #gets the translation
        translation_CL = self.translation
        translation_World = rotation_CL_to_world @ translation_CL

        #adds the tranlsation to the rotated vertices
        self.finalVertices = rotatedVertices + translation_World

        numVertices = np.shape(self.finalVertices)[1]


        #creates the 90 degree rotation matrix to the right
        Rotation_Right = np.array([[0.0, -1.0],
                                   [1.0, 0.0]])


        normalVectors_list = []

        vertices_list = []

        for i in range(numVertices):

            currentVertex = self.finalVertices[:,i:(i+1)]

            #appends the current vertex to the vertices list
            vertices_list.append(currentVertex)

            if i == (numVertices - 1):
                nextVertex = self.finalVertices[:,0:1]
            else:
                nextVertex = self.finalVertices[:,(i+1):(i+2)]

            #gets the vector from current to next
            vectorCurrentToNext = nextVertex - currentVertex

            #gets the normal of that
            vectorCurrentToNext_norm = vectorCurrentToNext / np.linalg.norm(vectorCurrentToNext)

            #gets the normal vector
            currentNormalVector = Rotation_Right @ vectorCurrentToNext_norm

            normalVectors_list.append(currentNormalVector)

    
        return normalVectors_list, vertices_list


    #does the same thing for the 3-Dimensional case
    def __getNormalsVertices_3d(self):
        #gets the size of the x dimension
        x_dimension = self.dimensions[0,0]
        #gets the size of the y dimensions
        y_dimension = self.dimensions[1,0]
        #same for z
        z_dimension = self.dimensions[2,0]

        #gets the mins and maxes for the x y and z
        x_min = -x_dimension/2.0
        x_max = x_dimension/2.0

        y_min = -y_dimension/2.0
        y_max = y_dimension/2.0

        z_min = -z_dimension/2.0
        z_max = z_dimension/2.0

        initialVertices = np.array([[x_min, x_max, x_max, x_min, x_min, x_max, x_max, x_min],
                                         [y_min, y_min, y_max, y_max, y_min, y_min, y_max, y_max],
                                         [z_min, z_min, z_min, z_min, z_max, z_max, z_max, z_max]])

        #gets the two rotation matrices
        rotation_CL_to_world = self.rotation
        rotation_world_to_CL = rotation_CL_to_world.T

        #gets the rotated vertices
        rotatedVertices = rotation_CL_to_world @ initialVertices

        #gets the translation in the CL frame
        translation_CL = self.translation
        #gets the translatio nin the world grame
        translation_World = rotation_CL_to_world @ translation_CL

        #adds the translation t othe rotated vertices
        finalVertices = rotatedVertices + translation_World

        #calls the function to get the 3d vertices and nromals
        normalVectors_list, verticesForNormalVectors_list = self.generate3DNormalsVertices(vertices=finalVertices)

        return normalVectors_list, verticesForNormalVectors_list



    #defines the function to get the normal vectors in 3D
    def generate3DNormalsVertices(self,
                                  vertices: np.ndarray):


        #creates the list of the edge vectors
        self.edgeLengths = []
        self.edgeVectors = []
        #creates the end
        for edgeVertices in edges_verticesIndices_3D:

            #gets the indices for the two vertices
            startVertexIndex = edgeVertices[0]
            endVertexIndex = edgeVertices[1]

            #gets the corresponding vertices
            startVertex_pos = vertices[:,startVertexIndex:(startVertexIndex+1)]
            endVertex_pos = vertices[:,endVertexIndex:(endVertexIndex+1)]

            #gets the edge vector
            currentEdgeVector = endVertex_pos - startVertex_pos
            
            currentEdgeVector_length = np.linalg.norm(currentEdgeVector)
            #gets it normalized
            currentEdgeVector_norm = currentEdgeVector / currentEdgeVector_length
            #appends the unit vector
            self.edgeVectors.append(currentEdgeVector_norm)
            self.edgeLengths.append(currentEdgeVector_length)


        edgeVectorPairsIndices_list = [[1,0],
                                       [0,2],
                                       [3,4],
                                       [5,6],
                                       [2,1],
                                       [8,9]]                                

        #creates the normals vectors list
        normalVectorsList = []

        #creates the list of the points that correspoints to each normal vector
        verticesForNormalVectorsList = []

        #ok. now with the edge vectors list, we can go back through and use cross products 
        # that define the exterior-facing vectors
        for edgeVectorPairIndices in edgeVectorPairsIndices_list:

            #gets the corresponding position, which is just the position
            #of the 

            #gets the primary vector
            primaryVector_index = edgeVectorPairIndices[0]
            secondaryVector_index = edgeVectorPairIndices[1]

            #Gets the primary and the secondary vectors
            primaryVector = self.edgeVectors[primaryVector_index]
            secondaryVector = self.edgeVectors[secondaryVector_index]

            #gets the primary vector shape
            primaryVector_shape = np.shape(primaryVector)

            #gets the flattened versions of the primary and secondary vectors
            primaryVector_flattened = primaryVector.flatten()
            secondaryVector_flattened = secondaryVector.flatten()

            #gets the cross product of the primary and secondary vectors
            normalVector_temp_flattened = np.linalg.cross(primaryVector_flattened, secondaryVector_flattened)
            #reshapes the normal vector
            normalVector_temp = np.reshape(normalVector_temp_flattened, primaryVector_shape)
            #appends to the normal Vectors list
            normalVectorsList.append(normalVector_temp)

            #gets the edge vertices for the primary vector
            primaryVectorVertices_indices = edges_verticesIndices_3D[primaryVector_index]
            #gets the start position for the primary vector
            primaryVector_startPositionIndex = primaryVectorVertices_indices[0]
            #finally gets the primary vector start position
            primaryVector_startPosition = vertices[:,primaryVector_startPositionIndex:(primaryVector_startPositionIndex+1)]
            #appends to the vertices for normal vectors list
            verticesForNormalVectorsList.append(primaryVector_startPosition)


        return normalVectorsList, verticesForNormalVectorsList
    


