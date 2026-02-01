#plots the obstacles for the map, and the path itself using matplotlib
#instead of OpenGL, which is used for the actual simulations. This should look
#a lot nicer.
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
import rrt_mavsim.parameters.plotter_parameters as PLOT


class PlotMapPath:

    def __init__(self,
                 map: MsgWorldMap,
                 waypoints_not_smooth: MsgWaypoints_SFC = None,
                 waypoints_smooth: MsgWaypoints_SFC = None):
        self.map = map
        self.waypoints_not_smooth = waypoints_not_smooth
        self.waypoints_smooth = waypoints_smooth


    def plotMap(self):

        obstaclesList = self.map.get_obstacles()

        self.obstaclePositionsList_unrotated = []
        self.obstaclePositionsList_rotated = []
        self.meshes_obstacles = []

        for obstacle in obstaclesList:

            #gets the obstacle positions
            obstaclePosition = obstacle.getTranslationWorld()
            self.obstaclePositionsList_unrotated.append(obstaclePosition)

            #gets the points rotated from the NED frame to the altitude frame
            rotatedObstaclePosition = PLOT.R_NED_to_Altitude @ obstaclePosition
            self.obstaclePositionsList_rotated.append(rotatedObstaclePosition)

            #gets the object vertices
            obstacleVertices_unrotated = obstacle.getVertices_obstacle_3D_list()

            obstacleVertices_rotated = []

            #rotates the positioning of the obstacle vertices so that down is up.
            for obstacleVertex in obstacleVertices_unrotated:

                rotatedVertex = PLOT.R_NED_to_Altitude @ obstacleVertex
                obstacleVertices_rotated.append(rotatedVertex)

            #gets the obstacle meshes
            obstacleMeshes = self.getObstacleMeshes(vertices=obstacleVertices_rotated)
            self.meshes_obstacles.append(obstacleMeshes)
        

        testPoint = 0

    def getObstacleMeshes(self,
                          vertices: list[np.ndarray]):

            

        meshes = [[vertices[0], vertices[1], vertices[2]],
                  [vertices[0], vertices[2], vertices[3]],
                  [vertices[0], vertices[5], vertices[4]],
                  [vertices[0], vertices[1], vertices[5]],
                  [vertices[1], vertices[6], vertices[5]],
                  [vertices[1], vertices[2], vertices[6]],
                  [vertices[2], vertices[6], vertices[7]],
                  [vertices[2], vertices[7], vertices[3]],
                  [vertices[3], vertices[4], vertices[7]],
                  [vertices[3], vertices[0], vertices[4]],
                  [vertices[4], vertices[5], vertices[6]],
                  [vertices[4], vertices[6], vertices[7]]]


        return meshes
