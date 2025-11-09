import numpy as np
import pyqtgraph.opengl as gl
from bsplinegenerator.bsplines import BsplineEvaluation
#class to draw the control points of the subject

R = np.array([[0, 1, 0],
              [1, 0, 0],
              [0, 0, -1]])


class DrawTrajectory:

    #creates the init function
    def __init__(self,
                 controlPoints: np.ndarray,
                 sampledPoints_spline: np.ndarray,
                 window: gl.GLViewWidget,
                 lineColor: np.ndarray,
                 lineWidth: float,
                 pointWidth: float):


        #gets the control points and the spline points in the altitude frame
        controlPoints_altitudeFrame = R @ controlPoints
        sampledPoints_spline_altitudeFrame = R @ sampledPoints_spline

        #plots them out. 
        self.plotPath(lineColor=lineColor,
                      points=sampledPoints_spline_altitudeFrame.T,
                      window=window,
                      lineWidth=lineWidth)
        
        self.plot_scatter_ctrl_pts(pointColor=lineColor,
                                   points=controlPoints_altitudeFrame.T,
                                   window=window,
                                   pointWidth=pointWidth)

        
    #defines the helpber function to add the plot object to the window
    #This will help us clean up the code a little bit more.
    def plotPath(self, 
                 lineColor: np.ndarray,
                 points: np.ndarray,
                 window: gl.GLViewWidget,
                 lineWidth: float):
        waypoint_color = np.tile(lineColor, (points.shape[0], 1))
        self.waypoint_plot_object = gl.GLLinePlotItem(pos=points,
                                                      color=waypoint_color,
                                                      width=lineWidth,
                                                      antialias=False,
                                                      mode='line_strip')
        self.waypoint_plot_object.setGLOptions('translucent')  # puts waypoint behind obstacles
        # ============= options include
        # opaque        Enables depth testing and disables blending
        # translucent   Enables depth testing and blending
        #               Elements must be drawn sorted back-to-front for
        #               translucency to work correctly.
        # additive      Disables depth testing, enables blending.
        #               Colors are added together, so sorting is not required.
        # ============= ======================================================
        window.addItem(self.waypoint_plot_object)

    #defines the function to scatter plot control points
    def plot_scatter_ctrl_pts(self,
                              pointColor: np.ndarray,
                              points: np.ndarray,
                              window: gl.GLViewWidget,
                              pointWidth: float):
        
        control_point_color = np.tile(pointColor, (points.shape[0], 1))

        self.waypointPositionsPlotObject = gl.GLScatterPlotItem(pos=points,
                                                                color=control_point_color,
                                                                size=pointWidth)
        
        #sets as translucent
        self.waypointPositionsPlotObject.setGLOptions('translucent')
        #adds this item to the window
        window.addItem(self.waypointPositionsPlotObject)