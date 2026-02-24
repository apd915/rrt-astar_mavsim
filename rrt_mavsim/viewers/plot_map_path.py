# plots the obstacles for the map, and the path itself using matplotlib
# instead of OpenGL, which is used for the actual simulations. This should look
# a lot nicer.
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
import rrt_mavsim.parameters.plotter_parameters as PLOT
from rrt_mavsim.message_types.msg_plane import MsgPlane
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from copy import deepcopy


class PlotMapPath:
    def __init__(
        self,
        map: MsgWorldMap,
        waypoints_not_smooth: MsgWaypoints_SFC = None,
        waypoints_smooth: MsgWaypoints_SFC = None,
        plane: MsgPlane = None,
    ):
        self.map = map
        self.waypoints_not_smooth = waypoints_not_smooth
        self.waypoints_smooth = waypoints_smooth
        self.plane = plane

    def plot(
        self, x_limits: tuple, y_limits: tuple, z_limits: tuple, aspectRatio: list
    ):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        self.plotMap(ax=ax)

        if self.waypoints_not_smooth is not None:
            self.plotWaypoints(ax=ax, waypoints=self.waypoints_not_smooth, color="red")

        if self.waypoints_smooth is not None:
            self.plotWaypoints(ax=ax, waypoints=self.waypoints_smooth, color="purple")

        # Set equal aspect ratio
        ax.set_box_aspect(aspectRatio)

        # Set limits
        ax.set_xlim(x_limits[0], x_limits[1])
        ax.set_ylim(y_limits[0], y_limits[1])
        ax.set_zlim(z_limits[0], z_limits[1])

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.show()

        # calls the plot map function

    def plotWaypoints(self, ax, waypoints: MsgWaypoints_SFC, color: str):
        # from the waypoints not smooth, we get the list of all safe flight corridors
        safeFlightCorridors_list = waypoints.getAllFlightCorridors()

        # iterates over all of the flight corridors
        for safeFlightCorridor in safeFlightCorridors_list:
            # gets the normals and vertices
            normalsList, vertices_list = safeFlightCorridor.getNormalsVertices_3D(
                plane=self.plane
            )

            verticesListCopy = deepcopy(vertices_list)

            # creates the augmented vertices list to add the first point in the vertices List duplicated to the ending point
            # we do this so as to complete the circuit for the drawing.
            verticesListCopy.append(verticesListCopy[0])

            # concatenates together the verticesList into an array so we can plot the positions out
            vertices = np.concatenate((verticesListCopy), axis=1)

            # gets the vertices rotated into the altitude frame (out of the NED frame merely for plotting purposes)
            vertRot = PLOT.R_NED_to_Altitude @ vertices

            x_component = vertRot[0, :]
            y_component = vertRot[1, :]
            z_component = vertRot[2, :]

            ax.plot(
                x_component,
                y_component,
                z_component,
                color=color,
                linewidth=2,
                zorder=10,
            )

            #

            testPoint = 0

        testPoint = 0

    def plotMap(self, ax):
        obstaclesList = self.map.get_obstacles()

        self.obstaclePositionsList_unrotated = []
        self.obstaclePositionsList_rotated = []
        self.meshes_obstacles = []

        for obstacle in obstaclesList:
            # gets the obstacle positions
            obstaclePosition = obstacle.getTranslationWorld()
            self.obstaclePositionsList_unrotated.append(obstaclePosition)

            # gets the points rotated from the NED frame to the altitude frame
            rotatedObstaclePosition = PLOT.R_NED_to_Altitude @ obstaclePosition
            self.obstaclePositionsList_rotated.append(rotatedObstaclePosition)

            # gets the object vertices
            obstacleVertices_unrotated = obstacle.getVertices_obstacle_3D_list()

            obstacleVertices_rotated = []

            # rotates the positioning of the obstacle vertices so that down is up.
            for obstacleVertex in obstacleVertices_unrotated:
                rotatedVertex = PLOT.R_NED_to_Altitude @ obstacleVertex
                obstacleVertices_rotated.append(rotatedVertex)

            # gets the obstacle meshes
            obstacleMeshes = self.getObstacleMeshes(vertices=obstacleVertices_rotated)
            self.meshes_obstacles.append(obstacleMeshes)

            # creates this obstacle
            tempObstacle_poly = Poly3DCollection(
                obstacleMeshes, alpha=1.0, facecolor="blue", edgecolor="k", zorder=1
            )
            ax.add_collection3d(tempObstacle_poly)

        testPoint = 0

    def getObstacleMeshes(self, vertices: list[np.ndarray]):
        # obtains the flattened vertices
        vert_flat = [vertex.flatten() for vertex in vertices]

        meshes = [
            [vert_flat[0], vert_flat[1], vert_flat[2], vert_flat[3]],
            [vert_flat[0], vert_flat[1], vert_flat[5], vert_flat[4]],
            [vert_flat[0], vert_flat[4], vert_flat[7], vert_flat[3]],
            [vert_flat[1], vert_flat[2], vert_flat[6], vert_flat[5]],
            [vert_flat[2], vert_flat[3], vert_flat[7], vert_flat[6]],
            [vert_flat[4], vert_flat[5], vert_flat[6], vert_flat[7]],
        ]
        return meshes
