import numpy as np
import parameters.planner_parameters as PLAN
from shapely.geometry import MultiPoint
from tools.obstacles import RectangularObstacle


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
        if self.numDimensions == 2:

            self.init_2D_map()

        elif self.numDimensions == 3:

            self.init_3D_map()
            potato = 0

    
    def init_2D_map(self):

        #creates the obstacleWidth
        obstacleWidth = self.obstacleWidthRatio * self.stepSize

        #creates the list of lists of obstacles
        self.obstacleList_2D = []
        
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

            self.obstacleList_2D.append(tempNorthList)
        
    #gets the lists of the 2D obstacles
    def get_2D_obstacles(self)->list[list[RectangularObstacle]]:
        return self.obstacleList_2D


    def init_3D_map(self):

        self.obstacleList_3D = []
        
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
                        
                    
                    tempEastList.append(tempObstacle)
                tempNorthList.append(tempEastList)
            self.obstacleList_3D.append(tempNorthList)


    def get_3D_obstacles(self)->list[list[list[RectangularObstacle]]]:
        return self.obstacleList_3D



    def getConvexHullsList(self):
        convexHullsList = []

        #case 2 dimensions
        if self.numDimensions == 2:

            tempList = self.get_2D_obstacles()
            
            #gets the vertices list
            for NorthList in tempList:
                for tempObject in NorthList:
                    
                    #gets the vertices
                    tempVertices = tempObject.getVertices_building_2D_list()

                    tempConvexHull = MultiPoint(tempVertices).convex_hull
                    convexHullsList.append(tempConvexHull)

        elif self.numDimensions == 3:

            tempList = self.get_3D_obstacles()

            for NorthList in tempList:
                for eastList in NorthList:
                    for tempObject in eastList:

                        tempVertices = tempObject.getVertices_obstacle_3D_list()

                        tempConvexHull = MultiPoint(tempVertices).convex_hull
                        convexHullsList.append(tempConvexHull)

        return convexHullsList
