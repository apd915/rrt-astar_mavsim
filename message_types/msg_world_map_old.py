import numpy as np
import parameters.planner_parameters as PLAN
from shapely.geometry import MultiPoint
from tools.obstacles import RectangularObstacle
import time


#creates the set of possible obstalcle types
obstacleTypes = ['rectangular', 'spherical', 'walls']


class MsgWorldMap:

    #initialization function
    def __init__(self,
                 obstacleFieldType: str,
                 numDimensions: int,
                 fieldWidth: float = PLAN.city_width,
                 obstacleWidthRatio: float = PLAN.obstacleWidthRatio,
                 obstacleWidth_sigma: float = PLAN.obstacleWidth_sigma,
                 altitude: float = PLAN.altitude,
                 numBlocks: int = PLAN.num_blocks):
        

        #saves all fo the above
        self.obstacleFieldType = obstacleFieldType
        self.numDimensions = numDimensions
        self.fieldWidth = fieldWidth
        self.obstacleWidthRatio = obstacleWidthRatio
        self.obstacleWidth_sigma = obstacleWidth_sigma
        self.altitude = altitude
        self.numBlocks = numBlocks

        #sets the step size
        self.stepSize = self.fieldWidth / self.numBlocks

        self.initPosition = self.stepSize / 2.0

        #sets the average obstacle size
        self.obsDim_mu = self.obstacleWidthRatio * self.stepSize

        #sets the maximum Obstacle Width
        self.maxObsWidth = self.stepSize * PLAN.maxObstacleWidthRatio
        self.minObsWidth = self.stepSize * PLAN.minObstacleWidthRatio


        #case this is a 2 dimensional obstacle
        self.init_map()

        #generates the A and b matrices list
        self.generateAbMatricesLists()



    def init_map(self):
        #calls the respective dimensionality to get the map
        if self.numDimensions == 2:
            self.init_2D_map()
        elif self.numDimensions == 3:
            self.init_3D_map()
    
    def init_2D_map(self):

        #creates the obstacleWidth
        obstacleWidth = self.obstacleWidthRatio * self.stepSize

        #creates the list of lists of obstacles
        self.obstacleList_2D = []

        #lists of center positions
        self.obstaclePositions_2D = []
        
        #iterates over the north positions
        for i in range(self.numBlocks):

            #gets the current northPosition
            northPosition = self.initPosition + i*self.stepSize

            #north temp list
            tempNorthList = []
            #iterates over the east position
            for j in range(self.numBlocks):
                
                eastPosition = self.initPosition + j*self.stepSize
                #gets the current position
                currentPosition = np.array([[northPosition],[eastPosition]])

                #generates the building height
                tempBuildingHeight = np.random.uniform(low=PLAN.building_height_min, high=PLAN.building_height_max)

                if self.obstacleFieldType == obstacleTypes[0]:
                    tempObstacle = RectangularObstacle(dimensions_obs=np.array([[obstacleWidth],[obstacleWidth]]),
                                                       translation_obs=currentPosition,
                                                       rotation_obsToWorld=np.eye(2),
                                                       building_height=tempBuildingHeight)
                    tempNorthList.append(tempObstacle)

                    self.obstaclePositions_2D.append(tempObstacle.getTranslationWorld())

            self.obstacleList_2D.append(tempNorthList)
        
    #gets the lists of the 2D obstacles
    def get_2D_obstacles(self)->list[list[RectangularObstacle]]:
        return self.obstacleList_2D


    def init_3D_map(self):

        self.obstacleList_3D = []

        self.obstaclePositions_3D = []
        
        for i in range(self.numBlocks):

            northPosition = self.initPosition + i*self.stepSize
            tempNorthList = []

            for j in range(self.numBlocks):

                eastPosition = self.initPosition + j*self.stepSize
                tempEastList = []

                for k in range(self.numBlocks):
                    #TODO. Make sure it's negative and not positive
                    downPosition = -(self.initPosition + k*self.stepSize)

                    currentPosition = np.array([[northPosition],[eastPosition],[downPosition]])

                    #creates the temp Dimensions for this object
                    tempDimensions = np.array([[0.0],[0.0],[0.0]])

                    #randomizes on the different dimensions
                    for DimCount in range(3):
                        #gets randomized shapes for the obstacles
                        tempBuildingDim = np.random.normal(loc=self.obsDim_mu,
                                                              scale=PLAN.obstacleWidth_sigma)
                        #clips the temp building length
                        tempBuildingDim = np.clip(tempBuildingDim, a_min=self.minObsWidth, a_max=self.maxObsWidth)
                        #sets the temp Building dimension to  the dim
                        tempDimensions[DimCount,0] = tempBuildingDim

                    #creates the Object
                    if self.obstacleFieldType == obstacleTypes[0]:
                        tempObstacle = RectangularObstacle(dimensions_obs=tempDimensions,
                                                           translation_obs=currentPosition,
                                                           rotation_obsToWorld=np.eye(3))
                        
                        tempObstaclePosition = tempObstacle.getTranslationWorld()
                        self.obstaclePositions_3D.append(tempObstaclePosition)
                        
                    
                    tempEastList.append(tempObstacle)
                tempNorthList.append(tempEastList)
            self.obstacleList_3D.append(tempNorthList)


    def get_3D_obstacles(self)->list[list[list[RectangularObstacle]]]:
        return self.obstacleList_3D

    #gets all of the obstacles objects
    def get_obstacles(self)->(list[list[RectangularObstacle]] | list[list[list[RectangularObstacle]]]):

        if self.numDimensions == 2:
            return self.get_2D_obstacles()
        elif self.numDimensions == 3:
            return self.get_3D_obstacles()


    #gets all of the A and b matrices for each of the obstacles as a large list
    def generateAbMatricesLists(self):

        self.obstaclesList = self.get_obstacles()

        self.Ab_list = []

        if self.numDimensions == 2:

            #iterates over all the obstacles to get the A and b matrices
            for northList in self.obstaclesList:
                for tempObstacle in northList:
                    
                    #gets the SFC from the temp Obstacle
                    obstacleSFCTemp = tempObstacle.getSFC()
                    #gets the A and b matrices
                    A_temp, b_temp = obstacleSFCTemp.generateAbMatrices()
                    #appends to the north Ab list
                    self.Ab_list.append([A_temp, b_temp])

                    potato = 0
                
        if self.numDimensions == 3:

            for northList in self.obstaclesList:
                for eastList in northList:
                    for tempObstacle in eastList:

                        #gets the SFC from the temp obstacle
                        obstacleSFCTemp = tempObstacle.getSFC()
                        A_temp, b_temp = obstacleSFCTemp.getAbMatrices()

                        self.Ab_list.append([A_temp, b_temp])


    def getAbMatricesLists(self):
        return self.Ab_list
    

    def getCenterPositionsList(self):

        if self.numDimensions == 2:
            return self.obstaclePositions_2D
        elif self.numDimensions == 3:
            return self.obstaclePositions_3D