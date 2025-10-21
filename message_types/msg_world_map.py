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

        #case this is a 2 dimensional obstacle
        if self.numDimensions == 2:

            self.init_2D_map()

        elif self.numDimensions == 3:

            self.init_3D_map()

    
    def init_2D_map(self):
        
        #iterates over the north positions
        for i in range(self.numBlocks):
            #iterates over the east position
            for j in range(self.numBlocks):
                
                #gets the current position
                currentPosition = np.array([[],[]])

                if self.obstacleFieldType == obstacleTypes[0]:
                    obstacle = RectangularObstacle(dimensions_obs=np.array([[10.0],[100.0]]),
                                                   translation_obs=np.array([[0.0],[0.0]]),
                                                   rotation_obsToWorld=np.array([[1.0, 0.0],
                                                                                 [0.0, 1.0]]))
        
        potato = 0

    def init_3D_map(self):

        pass


