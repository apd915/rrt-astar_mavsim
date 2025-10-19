import numpy as np
import parameters.planner_parameters as PLAN
from shapely.geometry import MultiPoints



class MsgWorldMap:

    #initialization function
    def __init__(self,
                 numDimensions: int,
                 fieldWidth: float = PLAN.city_width,
                 obstacleWidthRatio: float = PLAN.obstacleWidthRatio,
                 obstacleWidth_sigma: float = PLAN.obstacleWidth_sigma,
                 altitude: float = PLAN.altitude,
                 numBlocks: int = PLAN.num_blocks):
        

        #saves all fo the above
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
        pass

    def init_3D_map(self):

        pass

    
