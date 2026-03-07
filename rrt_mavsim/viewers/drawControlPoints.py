
#defines the function to draw the waypoints 
import numpy as np
import pyqtgraph.opengl as gl
import rrt_mavsim.parameters.display_parameters as DISPLAY
from rrt_mavsim.message_types.msg_plane import MsgPlane


R = np.array([[0, 1, 0], 
              [1, 0, 0], 
              [0, 0, -1]])



red = (1.0, 0.0, 0.0, 1.0)
purple = (170/255, 0, 1.0, 1.0)

class DrawControlPoints:


    def __init__(self,
                 controlPoints_list: list[np.ndarray],
                 window: gl.GLViewWidget,
                 plane: MsgPlane,
                 R_ned_to_alt: np.ndarray,
                 lineColor: tuple = red):

        #saves the Rotation matrix, which is the rotation from NED to altitude frame
        self.R_ned_to_alt = R_ned_to_alt

        #gets the dimension
        numDimensions = np.shape(controlPoints_list[0])[0]


        self.plane = plane

        for controlPointArray in controlPoints_list:

            if numDimensions == 2:

                self.drawControlPoints_2D()


    #defines how to draw control points in 2 dimensions
    def drawControlPoints_2D(self,
                             controlPoints: np.ndarray,
                             color: np.ndarray,
                             lineWidth: float,
                             pointWidth: float,
                             window: gl.GLViewWidget)


        #gets the control points in the altitude frame
        controlPoints_altitude = self.R_ned_to_alt @ controlPoints

    #defines the draw 2 dimensions thing
    def drawSFC_2D(self,
                   flightCorridor: MsgFlightCorridor,
                   color: np.ndarray,
                   lineWidth: float,
                   window: gl.GLViewWidget):
        
        #gets the sfc from the flight corridor
        normalsList, vertices_list = flightCorridor.getNormalsVertices_3D(plane=self.plane)

        numVertices = len(vertices_list)

        for i in range(numVertices):

            currentVertex = vertices_list[i]
            nextVertex = vertices_list[(i+1)%numVertices]

            #gets them in the rotated frame
            currentVertex_rotated = self.R_ned_to_alt @ currentVertex
            nextVertex_rotated = self.R_ned_to_alt @ nextVertex

            #gets the edge concatenateion
            edge_concatenated = np.concatenate((currentVertex_rotated.T, nextVertex_rotated.T), axis=0)
            
            #creates the lineplot item
            linePlot = gl.GLLinePlotItem(pos=edge_concatenated,
                                           color=color,
                                           width=lineWidth,
                                           antialias=True,
                                           mode='line_strip')
            linePlot.setGLOptions('opaque')

            #adds the item to the window
            window.addItem(item=linePlot)


        pass

    #defines the draw 3 dimensions thing
    def drawSFC_3D(self,
                   flightCorridor: MsgFlightCorridor):

        #gets the normals and vertices list
        normalsList, vertices_list = flightCorridor.getNormalsVertices_3D(plane=self.plane)
        

        pass

