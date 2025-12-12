#implements the funcitons to convert square obstacles into David Christensen's obstacle
#types

from eVTOL_BSplines.submodules.path_generator.path_generation.obstacle import Obstacle

from rrt_mavsim.tools.obstacles import RectangularObstacle
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_plane import MsgPlane


def worldMap_rectToCircle_conversion(worldMap: MsgWorldMap,
                                     plane: MsgPlane):

    obstacleList_rect_3D = worldMap.obstaclesList

    circularObstacleList = []

    for rect_obstacle_3D in obstacleList_rect_3D:

        circular_obstacle = rect_obstacle_3D.getCircularObstacle(plane_msg=plane)

        

        circularObstacleList.append(circular_obstacle)
    
    return circularObstacleList