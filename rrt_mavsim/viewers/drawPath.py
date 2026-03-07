import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
import rrt_mavsim.parameters.display_parameters as DISPLAY
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D
from bsplinegenerator.bsplines import BsplineEvaluation


#defines the class to draw the control points
class DrawPath:

    def __init__(self,
                 degree: int,
                 numDimensions: int,
                 R_ned_to_alt: np.ndarray,
                 plane: MsgPlane = None,
                 controlPoints_list: list[np.ndarray] = None,
                 window: gl.GLViewWidget = None,
                 color_list: list[np.ndarray] = None,
                 lineWidth: float = None,
                 pointWidth: float = None,
                 numDataPointsPerInterval: int = 100):
        
        self.numDataPointsPerInterval = numDataPointsPerInterval
        self.R_ned_to_alt = R_ned_to_alt
        self.plane = plane

        if numDimensions == 2:
            self.drawPath_2D(degree=degree,
                             controlPoints_list=controlPoints_list,
                             color_list=color_list,
                             lineWidth=lineWidth,
                             pointWidth=pointWidth,
                             window=window)


    def drawPath_2D(self,
                    degree: int,
                    controlPoints_list: list[np.ndarray],
                    color_list: list[np.ndarray],
                    lineWidth: float,
                    pointWidth: float,
                    window: gl.GLViewWidget):


        for controlPointsArray, color in zip(controlPoints_list, color_list):

            #cretes the BSpline object
            bspline = BsplineEvaluation(control_points=controlPointsArray,
                                        order=degree,
                                        start_time=0.0)

            #gets the sampled points from the Bspline
            bspline_sampled_points, bspline_time_data = bspline.get_spline_data(num_data_points_per_interval=self.numDataPointsPerInterval)

            #get the sampled points expressed in 3D
            bspline_sampled_points_3D = map_2D_to_3D(vec_2D=bspline_sampled_points,
                                                              plane=self.plane)

            controlPoints_3D = map_2D_to_3D(vec_2D=controlPointsArray,
                                                     plane=self.plane)
            #gets the bplsine sampled points and rotates them into the altiude frame
            bspline_sampled_points_altitude = self.R_ned_to_alt @ bspline_sampled_points_3D
            controlPoints_altitude = self.R_ned_to_alt @ controlPoints_3D



            #calls the function to plot a path
            self.drawSpline(lineColor=color,
                            sampledPoints=bspline_sampled_points_altitude,
                            window=window,
                            lineWidth=lineWidth)

            self.plot_scatter_ctrl_pts(pointColor=color,
                                       points=controlPoints_altitude,
                                       window=window,
                                       pointWidth=pointWidth)


    def drawSpline(self,
                   lineColor: np.ndarray,
                   sampledPoints: np.ndarray,
                   window: gl.GLViewWidget,
                   lineWidth: float):
        
        
        sampledPoints_transposed = sampledPoints.T

        waypoint_color = np.tile(lineColor, (sampledPoints_transposed.shape[0], 1))
        self.waypoint_plot_object = gl.GLLinePlotItem(pos=sampledPoints_transposed,
                                                      color=waypoint_color,
                                                      width=lineWidth,
                                                      antialias=False,
                                                      mode='line_strip')
        self.waypoint_plot_object.setGLOptions('opaque')  # puts waypoint behind obstacles


        window.addItem(self.waypoint_plot_object)

    #defines the function to scatter plot control points
    def plot_scatter_ctrl_pts(self,
                              pointColor: np.ndarray,
                              points: np.ndarray,
                              window: gl.GLViewWidget,
                              pointWidth: float):

        #gets the transpose of the points
        points_transposed = points.T
        
        control_point_color = np.tile(pointColor, (points_transposed.shape[0], 1))

        self.waypointPositionsPlotObject = gl.GLScatterPlotItem(pos=points_transposed,
                                                                color=control_point_color,
                                                                size=pointWidth)
        
        #sets as translucent
        self.waypointPositionsPlotObject.setGLOptions('opaque')
        #adds this item to the window
        window.addItem(self.waypointPositionsPlotObject)


        
