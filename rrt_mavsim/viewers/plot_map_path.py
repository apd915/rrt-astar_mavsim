# plots the obstacles for the map, and the path itself using matplotlib
# instead of OpenGL, which is used for the actual simulations. This should look
# a lot nicer.
from cvxpy import vec
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
import rrt_mavsim.parameters.plotter_parameters as PLOT
from rrt_mavsim.message_types.msg_plane import MsgPlane
from bsplinegenerator.bsplines import BsplineEvaluation
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D
from copy import deepcopy

color_list = ['green','yellow','orange']

#creates the list of points for each side
sideLists = [[0,1,2,3,0],#-75 3
             [0,3,7,4,0],#-30 1
             [0,1,5,4,0],#-75 2
             [4,5,6,7,4],#75 3
             [1,2,6,5,1],#730 1
             [2,3,7,6,2]]#-75 2

class PlotMapPath:
    def __init__(
        self,
        map: MsgWorldMap,
        waypoints_not_smooth: MsgWaypoints_SFC | None = None,
        waypoints_smooth: MsgWaypoints_SFC | None = None,
        controlPoints_not_smooth_list: list[np.ndarray] | None= None,
        controlPoints_smooth_list: list[np.ndarray]  | None = None,
        plane: MsgPlane | None = None,
        degree: int = 3,
    ):
        self.map = map
        self.waypoints_not_smooth = waypoints_not_smooth
        self.waypoints_smooth = waypoints_smooth
        self.controlPoints_not_smooth_list = controlPoints_not_smooth_list
        self.controlPoints_smooth_list = controlPoints_smooth_list
        self.plane = plane
        self.degree=degree

        self.numSamplesPerSegment = 100

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


        if self.controlPoints_not_smooth_list is not None:

            for controlPoints, color in zip(self.controlPoints_not_smooth_list, color_list):
                #evalueates the control points not smooth for a sampling
                #iterates over all of the control points lists
                bspline_not_smooth = BsplineEvaluation(control_points=controlPoints,
                                                       order=self.degree,
                                                       start_time=0.0)

                notSmooth_samples, time_notSmooth = bspline_not_smooth.get_spline_data(num_data_points_per_interval=self.numSamplesPerSegment)

                self.plotTrajectory(ax=ax,
                                    controlPoints=controlPoints,
                                    spline_sampled_points=notSmooth_samples,
                                    color=color)
            
        if self.controlPoints_smooth_list is not None:
            for controlPoints, color in zip(self.controlPoints_smooth_list, color_list):
                #evalueates the control points not smooth for a sampling
                bspline_smooth = BsplineEvaluation(control_points=controlPoints,
                                                       order=self.degree,
                                                       start_time=0.0)

                smooth_samples, time_notSmooth = bspline_smooth.get_spline_data(num_data_points_per_interval=self.numSamplesPerSegment)

                self.plotTrajectory(ax=ax,
                                    controlPoints=controlPoints,
                                    spline_sampled_points=smooth_samples,
                                    color=color)


        # Set equal aspect ratio
        ax.set_box_aspect(aspectRatio)

        # Set limits
        ax.set_xlim(x_limits[0], x_limits[1])
        ax.set_ylim(y_limits[0], y_limits[1])
        ax.set_zlim(z_limits[0], z_limits[1])

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.view_init(elev=0.0,azim=0.0)
        plt.show(block=False)
        plt.savefig('Bad Path.png', dpi=300)

        # calls the plot map function

    def plotWaypoints(self, ax, waypoints: MsgWaypoints_SFC, color: str):

        numDimensions = waypoints.numDimensions
        if numDimensions == 2:
            self._plotWaypoints_2D(ax=ax, waypoints=waypoints, color=color)
        elif numDimensions == 3:
            self._plotWaypoints_3D(ax=ax, waypoints=waypoints, color=color)


        testPoint = 0

    def _plotWaypoints_2D(self, ax, waypoints: MsgWaypoints_SFC, color: str):

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
        pass

    def _plotWaypoints_3D(self, ax, waypoints: MsgWaypoints_SFC, color: str):

        safeFlightCorridors_list = waypoints.getAllFlightCorridors()

        for safeFlightCorridor in safeFlightCorridors_list:
            verticesArray = safeFlightCorridor.sfc.getAllVertices_3D()
            numVertices = np.shape(verticesArray)[1]

            verticesList = [verticesArray[:,i:(i+1)] for i in range(numVertices)]

            #creates all of the sides
            sideVertexLists = []

            for side in sideLists:

                tempSide = []

                for index in side:

                    tempVertex = verticesList[index]
                    tempSide.append(tempVertex)

                sideVerticesArray = np.concatenate((tempSide), axis=1)

                #gets them rotated into the altitude frame
                sideVerticesArray_rotated = PLOT.R_NED_to_Altitude @ sideVerticesArray

                #plots the side
                x_component = sideVerticesArray_rotated[0,:]
                y_component = sideVerticesArray_rotated[1,:]
                z_component = sideVerticesArray_rotated[2,:]

                #plots this sode out
                ax.plot(
                    x_component,
                    y_component,
                    z_component,
                    color=color,
                    linewidth=2,
                    zorder=1,
                )




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


    def plotTrajectory(self, ax, controlPoints: np.ndarray, spline_sampled_points: np.ndarray, color: str):


        #gets the dimensionality of the control points
        controlPoints_shape = np.shape(controlPoints)
        numDimensions_controlPoints = controlPoints_shape[0]
        numControlPoints = controlPoints_shape[1]

        #case, the control points are 2D
        if numDimensions_controlPoints == 2:
            #converts the control points and sampled points to 3D
            controlPoints_3D = map_2D_to_3D(vec_2D=controlPoints,
                                            plane=self.plane)
        elif numDimensions_controlPoints == 3:
            controlPoints_3D = controlPoints

        else:
            controlPoints_3D = controlPoints

        #gets them in the altitude frame
        controlPoints_3D_altitude = PLOT.R_NED_to_Altitude @ controlPoints_3D

        #gets the x y and z components of the control points
        ctrl_x = controlPoints_3D_altitude[0,:]
        ctrl_y = controlPoints_3D_altitude[1,:]
        ctrl_z = controlPoints_3D_altitude[2,:]


        #scatterPlots the control points
        ax.scatter(ctrl_x,
                   ctrl_y,
                   ctrl_z,
                   color=color,
                   s=2,
                   zorder=10)
        
        #checks if the sampled points are 2D, and if so, maps them to 3D
        if numDimensions_controlPoints == 2:

            spline_sampled_points_3D = map_2D_to_3D(vec_2D=spline_sampled_points,
                                                    plane=self.plane)
        else:
            spline_sampled_points_3D = spline_sampled_points

        spline_sampledPoints_3D_altitude = PLOT.R_NED_to_Altitude @ spline_sampled_points_3D

        spline_x = spline_sampledPoints_3D_altitude[0,:]
        spline_y = spline_sampledPoints_3D_altitude[1,:]
        spline_z = spline_sampledPoints_3D_altitude[2,:]

        ax.plot(spline_x,
                spline_y,
                spline_z,
                color=color,
                linewidth=1,
                zorder=10)

        pass

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
    
    def plot_astar(self, ax, path, voxel_resolution):
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

        # 2. Plot the A* Path
        path_x, path_y, path_z = zip(*path)
        res = voxel_resolution
        
        # Convert path indices to physical meters
        p_x_meters = np.array(path_x) * res
        p_y_meters = np.array(path_y) * res
        p_z_meters = np.array(path_z) * res

        # Draw the path as a thick red line
        ax.plot(p_x_meters, p_y_meters, p_z_meters, color='red', linewidth=3, label='A* Path')
        
        # Drop solid markers on the exact Start and Goal positions
        ax.scatter(p_x_meters[0], p_y_meters[0], p_z_meters[0], color='green', s=100, label='Start')
        ax.scatter(p_x_meters[-1], p_y_meters[-1], p_z_meters[-1], color='purple', s=100, label='Goal')

        # Formatting
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_zlabel('Z (Altitude)')
        ax.legend()

        plt.show()