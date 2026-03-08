
import rrt_mavsim.parameters.planner_parameters as PLAN
import numpy as np
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.planarVTOL_map_parameters as PLANAR_PARAM
import rrt_mavsim.parameters.city_parameters as CITY
import rrt_mavsim.parameters.planar_maze_parameters as PLANAR_MAZE
from rrt_mavsim.tools.obstacles import RectangularObstacle
from scipy.optimize import linprog
import time
from enum import Enum
from rrt_mavsim.tools.getEulerUnit import getRotFromUnitVec, getRotFromPlane
from rrt_mavsim.tools.plane_projections_2 import map_3D_to_2D, map_2D_to_3D, projectPosition_toPlane
from rrt_mavsim.message_types.msg_plane import MsgPlane


#creates the enumeration for 

class MapTypes(str, Enum):
    CITY = 'City'
    FLOATING_BLOCKS = 'FloatingBlocks'
    PLANAR_VTOL = 'PlanarVTOL'
    MAZE = 'Maze'
    PLANAR_VTOL_SIMPLIFIED = 'PlanarVTOLSimplified'
    PLANAR_WINDOW = 'PlanarWindow'
    PLANAR_MAZE = 'PlanarMaze'

class CityParams:

    def __init__(self,
                 plane: MsgPlane,
                 cityWidth: float = CITY.city_width,
                 numBlocks: int = CITY.num_blocks,
                 obstacleWidthRatio: float = CITY.obstacleWidthRatio,
                 startPosition: np.ndarray = CITY.startPosition_2D,
                 endPosition: np.ndarray = CITY.endPosition_2D):

        self.plane = plane
        
        self.cityWidth = cityWidth
        self.numBlocks = numBlocks
        self.obstacleWidthRatio = obstacleWidthRatio
        self.startPosition = startPosition
        self.endPosition = endPosition

        blockWidth = cityWidth / numBlocks
        obstacleWidth = obstacleWidthRatio * blockWidth


        #creates the obstacle dimensions
        self.obstacleDimensions = np.array([[obstacleWidth],
                                            [obstacleWidth],
                                            [CITY.building_height]])

        
        northStart = blockWidth / 2.0
        eastStart = blockWidth / 2.0

        #from this information, we create the 2D position of the obstacles
        self.obstaclePositions_list = []
        for i in range(numBlocks):
            currentNorth = northStart + blockWidth*i
            for j in range(numBlocks):
                currentEast = eastStart + blockWidth*j

                obstaclePosition = np.array([[currentNorth],[currentEast]])
                self.obstaclePositions_list.append(obstaclePosition)

class PlanarVTOLParams:

    def __init__(self,
                 plane: MsgPlane,
                 fieldLength: float = PLANAR_PARAM.mapLength,
                 fieldHeight: float = PLANAR_PARAM.mapHeight,
                 obstacleMaxWidth: float = PLANAR_PARAM.obstacleMaxWidth,
                 obstacleMinWidth: float = PLANAR_PARAM.obstacleMinWidth,
                 obstacleDepth: float = PLANAR_PARAM.obstacleDepth,
                 numObstacles: int = PLANAR_PARAM.numObstacles):
        
        self.fieldLength = fieldLength
        self.fieldHeight = fieldHeight
        self.obstacleMaxWidth = obstacleMaxWidth
        self.obstacleMinWidth = obstacleMinWidth
        self.obstacleDepth = obstacleDepth
        self.numObstacles = numObstacles


        #creates the dimensions of search
        #Note on the end altitude. Remember that in the 3D world frame, we define the positive
        #Z axis in the down direction. But I want the plane to end up at an altitude of 2000 units
        #so that means the search area will be freom the origin (0,0) to (0,2k) to (10k, -2k) to (10k,0)
        self.searchDimensions_3D = np.array([[0.0, self.fieldLength],
                                             [0.0, 0.0],
                                             [0.0, -self.fieldHeight]])
        
        self.startPosition = np.array([[0.0],[0.0],[0.0]])
        self.endPosition = np.array([[self.fieldLength],[0.0],[-self.fieldHeight]])

        self.searchDimension_3D_start = self.startPosition
        self.searchDimension_3D_end = self.endPosition


        self.searchDimension_3D_projected_start = projectPosition_toPlane(vec_3D_init=self.searchDimension_3D_start,
                                                                          plane=plane)

        self.searchDimension_3D_projected_end = projectPosition_toPlane(vec_3D_init=self.searchDimension_3D_end,
                                                                        plane=plane)

        self.searchDimensions_2D_start = map_3D_to_2D(vec_3D=self.searchDimension_3D_projected_start,
                                                      plane=plane)

        self.searchdimensions_2D_end = map_3D_to_2D(vec_3D=self.searchDimension_3D_projected_end,
                                                    plane=plane)
        
        potato = 0

class PlanarVTOLSimplifiedParams:

    def __init__(self,
                 plane: MsgPlane,
                 fieldLength: float = 2000.0,
                 fieldHeight: float = 1000.0,
                 obstacleWidth: float = 100.0,
                 obstacleDepth: float = 50.0,
                 numObstacles_north: int = 3,
                 numObstacles_down: int = 3):
        
        #save the plane message
        self.plane = plane

        stepSize_north = fieldLength / numObstacles_north
        startStepSize_north = stepSize_north / 2.0


        stepSize_down = fieldHeight / numObstacles_down
        startStepSize_down = stepSize_down / 2.0
        
        north_start = startStepSize_north
        down_start = -startStepSize_down

        #creates the dimensions of the individual obstacles (from the center as reference)
        #needs to be in the order of first depth, and then the variable dimensions 
        # (because this is in the order of )
        self.dimensions = np.array([[obstacleDepth],
                                    [obstacleWidth],
                                    [obstacleWidth]])
        
        #gets the search dimensions
        self.startPosition_2D = np.array([[0.0],[0.0]])
        #Please note that we are going up, so we need it to have a negative height
        self.endPosition_2D = np.array([[fieldLength],[-fieldHeight]])

        self.startPosition_3D = map_2D_to_3D(vec_2D=self.startPosition_2D,
                                             plane=self.plane)
        self.endPosition_3D = map_2D_to_3D(vec_2D=self.endPosition_2D,
                                           plane=self.plane)

        self.positionsList_2D = []
        #iterates over north
        for i in range(numObstacles_north):
            #iterates over altitude
            for j in range(numObstacles_down):
                currentNorth = north_start + i*stepSize_north
                currentDown = down_start - j*stepSize_down

                self.positionsList_2D.append(np.array([[currentNorth],[currentDown]]))

class FloatingBlocksParams:

    def __init__(self):

        #creates the 





        pass

class PlanarWindowParam:

    def __init__(self,
                 northLength: float = 1500.0,
                 downLength: float = 1000.0,
                 windowWidth: float = 300.0):

        self.northLength = northLength
        self.downLength = downLength
        self.windowWidth = windowWidth

        self.obstacleLength = (downLength-windowWidth)/2.0
        self.obstacleThickness = 50.0
        self.obstacleHeight = 100.0


class PlanarMazeParam:

    def __init__(self,
                 plane: MsgPlane = PLANAR_MAZE.plane_msg,
                 numTeeth: int = PLANAR_MAZE.numTeeth,
                 width: float = PLANAR_MAZE.width,
                 height: float = PLANAR_MAZE.height,
                 barHeight: float = PLANAR_MAZE.barHeight,
                 teethHeightRatio: float = PLANAR_MAZE.teethHeightRatio,
                 teethWidthRatio: float = PLANAR_MAZE.teethWidthRatio,
                 startPosition: np.ndarray = PLANAR_MAZE.startPosition,
                 endPosition: np.ndarray = PLANAR_MAZE.endPosition):

        self.plane = plane
        self.numTeeth = numTeeth
        self.numSpaces = int(self.numTeeth - 1)
        self.width = width
        self.height = height
        self.barHeight = barHeight
        self.teethHeightRatio = teethHeightRatio
        self.teethWidthRatio = teethWidthRatio
        self.startPosition = startPosition
        self.endPosition = endPosition

        #creates the obstacles
        spacingWidth = self.width / self.numSpaces
        teethHeight = self.teethHeightRatio*height
        teethWidth = spacingWidth*self.teethWidthRatio
        self.teethDimensions = np.array([[teethHeight],[teethWidth]])

        self.teethPositions = []

        for i in range(numTeeth):
            
            eastPosition = i*spacingWidth

            if i % 2 == 0:
                northPosition = height - (teethHeight/2.0)
            else:
                northPosition = teethHeight/2.0

            currentPosition = np.array([[northPosition],
                                        [eastPosition]])

            self.teethPositions.append(currentPosition)


        self.barWidth = width*((numTeeth-1.0)/numTeeth)

        self.barDimensions = np.array([[barHeight],
                                       [self.barWidth]])

        #creates the top and bottom bars
        self.bottomBarLocation = np.array([[-barHeight/2.0],
                                           [spacingWidth + self.barWidth/2.0]])

        self.topBarLocation = np.array([[self.height + barHeight/2.0],
                                        [self.barWidth/2.0]])

        self.startPosition = startPosition
        self.endPosition = endPosition



class MsgWorldMap:

    #creates the init function
    def __init__(self,
                 obstacleFieldType: MapTypes,
                 numDimensions_algorithm: int,
                 planarVTOL_Params: PlanarVTOLParams = None,
                 planarVTOLSimplified_Params: PlanarVTOLSimplifiedParams = None,
                 cityParams: CityParams = None,
                 planarMazeParams: PlanarMazeParam = None):
        

        #saves all fo the above
        self.obstacleFieldType = obstacleFieldType
        self.numDimensions_algorithm = numDimensions_algorithm

        #saves the parameters for each type of obstacle field
        self.pln_VTOL_Param = planarVTOL_Params
        self.planarVTOLSimplified_params = planarVTOLSimplified_Params
        self.cityParams = cityParams
        self.planarMazeParams = planarMazeParams

        if obstacleFieldType == MapTypes.PLANAR_VTOL:

            self.initPlanarVTOL_map()

            #sets the search dimensions
            self.searchDimensions_start = planarVTOL_Params.searchDimensions_2D_start
            self.searchDimensions_end = planarVTOL_Params.searchdimensions_2D_end


        elif obstacleFieldType == MapTypes.PLANAR_VTOL_SIMPLIFIED:

            self.initPlanarVTOLSimplified_map()

            self.searchDimensions_start = planarVTOLSimplified_Params.startPosition_2D
            self.searchDimensions_end = planarVTOLSimplified_Params.endPosition_2D
        elif obstacleFieldType == MapTypes.CITY:

            self.initCityMap()

            self.searchDimensions_start = cityParams.startPosition
            self.searchDimensions_end = cityParams.endPosition

        elif obstacleFieldType == MapTypes.PLANAR_MAZE:

            self.initPlanarMazeMap()
            self.searchDimensions_start = planarMazeParams.startPosition
            self.searchDimensions_end = planarMazeParams.endPosition

        self.generateAbMatricesLists()
        self.generateFeasibilityList()

        potato = 0

    def initCityMap(self):

        self.plane_msg = self.cityParams.plane

        Rot_subspaceToWorld = getRotFromPlane(plane=self.plane_msg)
        Rot_worldToSubspace = Rot_subspaceToWorld.T

        self.obstaclesList = []
        
        for position_2D in self.cityParams.obstaclePositions_list:

            #gets the 3D position
            position_3D = map_2D_to_3D(vec_2D=position_2D,
                                       plane=self.plane_msg)
            #gets the position in the subspace
            position_3D_subspace = Rot_worldToSubspace @ position_3D

            #creates the temp obstacle 
            temp_obstacle = RectangularObstacle(dimensions_obs=self.cityParams.obstacleDimensions,
                                                translation_obs=position_3D_subspace,
                                                rotation_obsToWorld=Rot_subspaceToWorld)

            self.obstaclesList.append(temp_obstacle)
    
    def initPlanarVTOL_map(self):

        #creates map of randomized obstacles for the 
        #in the 2D planar space, we get the bounds for x and y
        x_min = self.pln_VTOL_Param.mapOrigin_2D.item(0)
        z_min = self.pln_VTOL_Param.mapOrigin_2D.item(1)

        x_max = x_min + self.pln_VTOL_Param.fieldLength
        z_max = z_min + self.pln_VTOL_Param.fieldHeight

        #gets the Q basis matrix
        self.Q = getPlaneBasis(n_hat=self.pln_VTOL_Param.n_hat)

        self.p0 = self.pln_VTOL_Param.mapOrigin_3D

        #gets the Rotation matrix from the unit vector
        Rot_subspaceToWorld = getRotFromUnitVec(unitVec=self.pln_VTOL_Param.n_hat)

        #gets the rotation matrix from the world to the subspace rotation
        Rot_worldToSubspace = Rot_subspaceToWorld.T

        #gets the list of the obstacles
        self.obstaclesList = []


        #iterates over all of the obstacles in the section
        for _ in range(self.pln_VTOL_Param.numObstacles):
            

            #gets the randomized x and Z coordinates for the randomized position
            random_x = np.random.uniform(low=x_min, high=x_max)
            random_z = np.random.uniform(low=z_min, high=z_max)

            #sets the 2D position
            pos_random_2D = np.array([[random_x],[random_z]])

            #gets the random position in 3D world frame
            pos_random_3D_world = getWorldPos(pos_2D=pos_random_2D,
                                              Q=self.Q,
                                              p0=self.p0)

            #now we get a randomized shape for the obstacle
            random_length = np.random.uniform(low=self.pln_VTOL_Param.obstacleMinWidth,
                                              high=self.pln_VTOL_Param.obstacleMaxWidth)
            random_height  = np.random.uniform(low=self.pln_VTOL_Param.obstacleMinWidth,
                                              high=self.pln_VTOL_Param.obstacleMaxWidth)
            
            dimensions = np.array([[self.pln_VTOL_Param.obstacleDepth],[random_length],[random_height]])
            
            #gets the random position in the Planar frame
            pos_random_3D_subspace = Rot_worldToSubspace @ pos_random_3D_world

            #creates the rectangular object
            tempObstacle = RectangularObstacle(dimensions_obs=dimensions,
                                                  translation_obs=pos_random_3D_subspace,
                                                  rotation_obsToWorld=Rot_subspaceToWorld)

            #appends to the obstacle list
            self.obstaclesList.append(tempObstacle)

            #gets the temp SFC from the obstacle
            tempSFC = tempObstacle.getSFC()

            tomato = 0            

        potato = 0

    #'''
    def initPlanarVTOLSimplified_map(self):

        self.plane_msg = self.planarVTOLSimplified_params.plane


        #gets the rotation from the subspace to the world frame
        Rot_subspaceToWorld = getRotFromUnitVec(unitVec=self.plane_msg.n_hat)
        Rot_worldToSubspace = Rot_subspaceToWorld.T
        
        self.obstaclesList = []

        #iterates over all of the positions in the position list
        for position_2D in self.planarVTOLSimplified_params.positionsList_2D:

            #gets the position in 3 Dimensions in the world frame
            position_3D_world = map_2D_to_3D(vec_2D=position_2D,
                                             plane=self.plane_msg)

            #gets the position in the subspace frame
            position_3D_subspace = Rot_worldToSubspace @ position_3D_world

            #creates the Rectangular obstacle in this orientation
            tempObstacle = RectangularObstacle(dimensions_obs=self.planarVTOLSimplified_params.dimensions,
                                               translation_obs=position_3D_subspace,
                                               rotation_obsToWorld=Rot_subspaceToWorld)
            self.obstaclesList.append(tempObstacle)


        def initPlanarMaze(self,
                           )
    #'''
    def initPlanarMazeMap(self):

        #iterates over 
        self.plane_msg = self.planarMazeParams.plane

        self.obstaclesList = []

        Rot_subspaceToWorld = getRotFromPlane(plane=self.plane_msg)
        Rot_worldToSubspace = Rot_subspaceToWorld.T
        
        #iterates over all the teeth positions
        for position_2D in self.planarMazeParams.teethPositions:
            position_3D = map_2D_to_3D(vec_2D=position_2D,
                                       plane=self.plane_msg)

            #gets the position in the subspace
            position_3D_subspace = Rot_worldToSubspace @ position_3D


            #creates the temp obstacle 
            temp_toothObstacle = RectangularObstacle(dimensions_obs=self.planarMazeParams.teethDimensions,
                                                translation_obs=position_3D_subspace,
                                                rotation_obsToWorld=Rot_subspaceToWorld)

            self.obstaclesList.append(temp_toothObstacle)

        #adds the top and bottom bars
        bottomBar_position_3D = map_2D_to_3D(vec_2D=self.planarMazeParams.bottomBarLocation,
                                             plane=self.plane_msg)

        bottomBar_position_3D_subspace = Rot_worldToSubspace @ bottomBar_position_3D

        bottomBarObstacle = RectangularObstacle(dimensions_obs=self.planarMazeParams.barDimensions,
                                             translation_obs=bottomBar_position_3D_subspace,
                                             rotation_obsToWorld=Rot_subspaceToWorld)

        self.obstaclesList.append(bottomBarObstacle)

        topBar_position_3D = map_2D_to_3D(vec_2D=self.planarMazeParams.topBarLocation,
                                          plane=self.plane_msg)

        topBar_position_3D_subspace = Rot_worldToSubspace @ topBar_position_3D

        topBarObstacle = RectangularObstacle(dimensions_obs=self.planarMazeParams.barDimensions,
                                             translation_obs=topBar_position_3D_subspace,
                                             rotation_obsToWorld=Rot_subspaceToWorld)

        self.obstaclesList.append(topBarObstacle)


    #gets all of the A and b matrices for each of the obstacles as a large list
    def generateAbMatricesLists(self):

        self.Ab_3D_list = []

        self.Ab_2D_list = []

        for obstacle in self.obstaclesList:

            obstacleSFCTemp = obstacle.getSFC()

            A_3D_temp, b_3D_temp = obstacleSFCTemp.getAbMatrices()
            self.Ab_3D_list.append([A_3D_temp, b_3D_temp])

            #case this is a 2D map
            if self.numDimensions_algorithm == 2:
                Q = self.plane_msg.Q
                plane_origin_3D = self.plane_msg.origin_3D
                #gets the 2D equivalents
                A_2D = A_3D_temp @ Q
                b_2D = b_3D_temp - A_3D_temp @ plane_origin_3D
                self.Ab_2D_list.append([A_2D, b_2D])
                
    def get_obstacles(self):
        return self.obstaclesList
    
    #returns the 3D Ab list
    def get_Ab_3D(self):
        return self.Ab_3D_list
    
    #returns the 2D Ab list
    def get_Ab_2D(self):
        return self.Ab_2D_list
    
    #returns the Q matrix
    def get_Q(self):
        return self.Q
    

    def generateFeasibilityList(self):

        self.feasibilityList = []
        for Ab_pair in self.Ab_2D_list:

            A = Ab_pair[0]
            b = Ab_pair[1]

            dummyVariable = np.ones(A.shape[1])

            #checks the feasibility of this pair
            result = linprog(dummyVariable, A_ub=A, b_ub=b, method='highs')

            self.feasibilityList.append(result.success)





