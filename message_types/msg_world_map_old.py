
import numpy as np
import parameters.planner_parameters as PLAN
from shapely.geometry import MultiPoint

#creates the world map message
class MsgWorldMap:

    #init function
    def __init__(self,
                 numDimensions: int,
                 fieldWidth: float,
                 obstacleWidthRatio: float,
                 obstacleWidth_sigma: float,
                 minObstacleWidth: float,
                 altitude: float = PLAN.altitude,
                 numBlocks: int = PLAN.num_blocks):
        
        self.numDimensions = numDimensions
        self.fieldWidth = fieldWidth
        self.obstacleWidthRatio = obstacleWidthRatio
        self.obstacleWidth_sigma = obstacleWidth_sigma
        self.minObstacleWidth = minObstacleWidth
        self.altitude = altitude
        self.numBlocks = numBlocks


        if self.numDimensions == 2:

            self.init_2D_map()
        elif self.numDimensions == 3:
            
            self.init_3D_map()


    #defines the function to initialize the map, based on the number of dimensions
    def init_2D_map(self):

        #sets the width of the total block (including the empty space in between)
        self.totalBlock_width = self.fieldWidth / self.numBlocks

        #creates the maximum block width
        maxObstacleWidth = self.totalBlock_width * self.obstacleWidthRatio

        #gets the average obstacle width
        averageObstacleWidth = (maxObstacleWidth + self.minObstacleWidth) / 2.0




    #3D section

    #defines the function to initialize the map, based on the number of dimensions
    def init_3D_map(self):

        #sets the width of the total block (including the empty space in between)
        self.totalBlock_width = self.fieldWidth / self.numBlocks

        #creates the maximum block width
        maxObstacleWidth = self.totalBlock_width * self.obstacleWidthRatio

        #gets the average obstacle width
        averageObstacleWidth = (maxObstacleWidth + self.minObstacleWidth) / 2.0

        #creates the list of the obstacle widths
        self.buildingWidths_3D = self.obstacleWidth_sigma*np.random.randn(self.numBlocks, self.numBlocks, self.numBlocks) + averageObstacleWidth

        #clips the building widths to saturation
        self.buildingWidths_3D = np.clip(self.buildingWidths_3D, 
                                      a_min=self.minObstacleWidth, 
                                      a_max=maxObstacleWidth)

        #gets the positions list
        self.centerPositions_list = self.getObstacleCenterPositions()

        #gets the vertices
        self.verticesList = self.getAllVertices_3D(centerPositionsList=self.centerPositions_list,
                                                   obstacleWidths=self.buildingWidths_3D)
        
        self.allEdges = self.getAllEdges_3D(allVerticesList=self.verticesList)

        self.allNormals, self.allNormals_vertices, self.A_list, self.b_list \
            = self.getAllNormals_3D(allEdgesList=self.allEdges,
                                    allVerticesList=self.verticesList)
        

    #gets the center position of each of the obstacles
    def getObstacleCenterPositions(self):

        #gets the half width
        halfWidth = self.totalBlock_width / 2.0
        #gets the complete list
        obstaclesCenterPositions_list = []
        #
        for i in range(self.numBlocks):
            northList = []
            currentNorthPosition = halfWidth + i*self.totalBlock_width
            for j in range(self.numBlocks):
                eastList = []
                currentEastPosition = halfWidth + j*self.totalBlock_width
                for k in range(self.numBlocks):
                    currentDownPosition = halfWidth + k*self.totalBlock_width
                    #gets the temp position
                    tempPosition = np.array([[currentNorthPosition],
                                             [currentEastPosition],
                                             [currentDownPosition]])
                    #appends to the east list
                    eastList.append(tempPosition)
                #appends to the north list
                northList.append(eastList)
            #appends the north list to the obstacle list
            obstaclesCenterPositions_list.append(northList)
        
        #returns the positions list
        return obstaclesCenterPositions_list


    #defines the function to get the vertices for each of the obstacles given its center position
    def getAllVertices_3D(self,
                          centerPositionsList: list[list[list[np.ndarray]]],
                          obstacleWidths: np.ndarray):

        totalVerticesList = []

        for i, northList in enumerate(centerPositionsList):
            northList_output = []
            for j, eastList in enumerate(northList):
                eastList_output = []
                for k, centerPosition in enumerate(eastList):

                    #gets the current obstacle width
                    currentObstacleWidth = obstacleWidths[i,j,k]

                    #calls the get individual obstacle vertices
                    currentObstacleVertices = self.getIndividualObstacleVertices_3D(centerPosition=centerPosition,
                                                                            obstacleWidth=currentObstacleWidth)

                    #appends the current Obstacle Vertices to the ease list
                    eastList_output.append(currentObstacleVertices)
                #appends the east list ot the north list
                northList_output.append(eastList_output)
            #appends the north list to the total vertices list
            totalVerticesList.append(northList_output)

        #returns all the vertices
        return totalVerticesList


    #defines the function to get the vertices of an individual obstacle
    def getIndividualObstacleVertices_3D(self,
                                         centerPosition: np.ndarray,
                                         obstacleWidth: float):

        #gets the half width
        hw = obstacleWidth / 2.0

        #creates the vertices
        vertices_unshift = np.array([[-hw,  hw,  hw, -hw, -hw,  hw,  hw, -hw],
                                     [-hw, -hw,  hw,  hw, -hw, -hw,  hw, hw],
                                     [-hw, -hw, -hw, -hw,  hw,  hw,  hw, hw]])

        numVertices = np.shape(vertices_unshift)[1]

        shiftedVertices = []

        for i in range(numVertices):
            #gets the current vertex
            currentVertex_unshifted = vertices_unshift[:,i:(i+1)]
            currentVertex = currentVertex_unshifted + centerPosition

            shiftedVertices.append(currentVertex)

        return shiftedVertices
    
    #defines the function to get all the edges
    def getAllEdges_3D(self,
                    allVerticesList: list[list[list[list[np.ndarray]]]]):



        allEdgesList = []

        #iterates over all the vertices
        for i, northList in enumerate(allVerticesList):
            northList_output = []
            for j, eastList in enumerate(northList):
                eastList_output = []
                for k, currentVerticesList in enumerate(eastList):

                    edgesList_individual = []
                    for edges_verticesDoubleIndex in PLAN.edges_verticesIndices_3D:
                        #gets the start vertex
                        startVertex_index = edges_verticesDoubleIndex[0]
                        endVertex_index = edges_verticesDoubleIndex[1]

                        #gets the startVertex position
                        startVertex_position = currentVerticesList[startVertex_index]
                        endVertex_position = currentVerticesList[endVertex_index]

                        #gets the edge
                        currentEdge = endVertex_position - startVertex_position
                        currentEdgeNorm = currentEdge / np.linalg.norm(currentEdge)
                        edgesList_individual.append(currentEdgeNorm)
                    eastList_output.append(edgesList_individual)
                northList_output.append(eastList_output)
            allEdgesList.append(northList_output)

        #returns the all edges list
        return allEdgesList
    
    #gets the normals
    def getAllNormals_3D(self,
                      allEdgesList,
                      allVerticesList):



        #the list for all of the normals
        totalNormalVectorsList = []
        #the list for all of the vertices that the normals rest on
        totalVerticesForNormalsList = []

        #the list for all of the A matrices and b vectors (for linear conditions)
        totalAMatricesList = []
        totalbVectorsList = []

        #iterates over all north indices
        for northList_edges, northList_vertices in zip(allEdgesList, allVerticesList):
            northNormalsListOutput = []
            northVerticesListOutput = []
            northAList_output = []
            northbList_output = []
            #over east indices
            for eastList_edges, eastList_vertices in zip(northList_edges, northList_vertices):
                eastNormalsListOutput = []
                eastVerticesListOutput = []

                eastAList_output = []
                eastbList_output = []
                #over down indices
                for tempEdgesList, tempVerticesList in zip(eastList_edges, eastList_vertices):
                    currentObjectNormalsList = []
                    currentObjectVerticesList = []

                    #now that we have the list of edges for an individual object,
                    #we obtain the list of normals from that list
                    #iterates over the edge vector pairs indices list
                    for edgeVectorPairsIndices in PLAN.edgeVectorPairsIndices_list_3D:
                        #gets the primary vector
                        primaryVectorIndex = edgeVectorPairsIndices[0]
                        secondaryVectorIndex = edgeVectorPairsIndices[1]

                        #from the primary vector index, let us get the primary vertex
                        primaryVertices_indices = PLAN.edges_verticesIndices_3D[primaryVectorIndex]
                        #then we get
                        primaryVertex_index = primaryVertices_indices[0]
                        #gets the primary vertex (as in vertex position) from the index
                        primaryVertex = tempVerticesList[primaryVertex_index]
                        #appends the primary vertex to the current vertices list
                        currentObjectVerticesList.append(primaryVertex)

                        #getsthe primary and secondary vector
                        primaryVector = tempEdgesList[primaryVectorIndex]
                        secondaryVector = tempEdgesList[secondaryVectorIndex]

                        #gets the primary vector shape
                        primaryVector_shape = np.shape(primaryVector)

                        #gets the flattened versions
                        primaryVector_flatten = primaryVector.flatten()
                        secondaryVector_flatten = secondaryVector.flatten()

                        #gets the cross product
                        normalVector_flattened = np.cross(primaryVector_flatten, secondaryVector_flatten)

                        normalVector = np.reshape(normalVector_flattened, primaryVector_shape)

                        currentObjectNormalsList.append(normalVector)

                        potato = 0
                    eastVerticesListOutput.append(currentObjectVerticesList)
                    eastNormalsListOutput.append(currentObjectNormalsList)

                    #gets the A and b matrices
                    A, b = self.getAbMatrices(currentObjectNormals_list=currentObjectNormalsList,
                                         currentObjectVertices_list=currentObjectVerticesList)
                    #saves the A and b matrices
                    eastAList_output.append(A)
                    eastbList_output.append(b)
                    potato = 0

                northVerticesListOutput.append(eastVerticesListOutput)
                northNormalsListOutput.append(eastNormalsListOutput)

                #saves the a and b matrices
                northAList_output.append(eastAList_output)
                northbList_output.append(eastbList_output)
            totalVerticesForNormalsList.append(northVerticesListOutput)
            totalNormalVectorsList.append(northNormalsListOutput)

            totalAMatricesList.append(northAList_output)
            totalbVectorsList.append(northbList_output)

        #returns the total normal vectors list and the corresponding vertices list
        return totalNormalVectorsList, totalVerticesForNormalsList, totalAMatricesList, totalbVectorsList

    #function to get the A and b matrix for a collection of normals and vertices
    def getAbMatrices(self,
                      currentObjectNormals_list: list[np.ndarray],
                      currentObjectVertices_list: list[np.ndarray]):


        #gets the dimension of this thing
        normalVector_temp = currentObjectNormals_list[0]
        dimension = np.size(normalVector_temp)

        #creates the A and b matrices
        A = np.ndarray((0,dimension))
        b = np.ndarray((0,1))

        for normalVector, vertex in zip(currentObjectNormals_list, currentObjectVertices_list):

            A = np.concatenate((A, normalVector.T), axis=0)

            #gets the value for the b
            b_value = normalVector.T @ vertex
            b = np.concatenate((b, b_value), axis=0)

        return A, b


