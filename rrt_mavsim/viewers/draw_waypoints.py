#defines the function to draw the waypoints 
import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
import rrt_mavsim.parameters.display_parameters as DISPLAY


R = np.array([[0, 1, 0], 
              [1, 0, 0], 
              [0, 0, -1]])


red = np.array([[204, 0, 0],
                [204, 0, 0]])/255.


purple = np.array([[170, 0, 255],
                   [170, 0, 255]])/255


class DrawWaypoints:


    def __init__(self,
                 waypoints: MsgWaypoints_SFC,
                 window: gl.GLViewWidget,
                 lineColor: np.ndarray = red,
                 n_hat: np.ndarray = None,
                 p0: np.ndarray = None):


        #gets the dimension
        numDimensions = waypoints.numDimensions

        #gets the list of SFCs
        flightCorridor_list = waypoints.getAllFlightCorridors()

        self.n_hat = n_hat
        self.p0 = p0


        for flightCorridor in flightCorridor_list:

            if numDimensions == 2:
                self.drawSFC_2D(flightCorridor=flightCorridor,
                                color=lineColor,
                                lineWidth=DISPLAY.flightCorridor_lineWidth,
                                window=window)
                
            elif numDimensions == 3:

                self.drawSFC_3D(sfc=flightCorridor)


    #defines the draw 2 dimensions thing
    def drawSFC_2D(self,
                   flightCorridor: MsgFlightCorridor,
                   color: np.ndarray,
                   lineWidth: float,
                   window: gl.GLViewWidget):
        
        #gets the sfc from the flight corridor
        normalsList, vertices_list = flightCorridor.getNormalsVertices_3D(n_hat=self.n_hat,
                                                                          p0=self.p0)

        numVertices = len(vertices_list)

        for i in range(numVertices):

            currentVertex = vertices_list[i]
            nextVertex = vertices_list[(i+1)%numVertices]

            #gets them in the rotated frame
            currentVertex_rotated = R @ currentVertex
            nextVertex_rotated = R @ nextVertex

            #gets the edge concatenateion
            edge_concatenated = np.concatenate((currentVertex_rotated.T, nextVertex_rotated.T), axis=0)
            
            #creates the lineplot item
            linePlot = gl.GLLinePlotItem(pos=edge_concatenated,
                                           color=color,
                                           width=lineWidth,
                                           antialias=True,
                                           mode='line_strip')
            linePlot.setGLOptions('additive')

            #adds the item to the window
            window.addItem(item=linePlot)


        pass

    #defines the draw 3 dimensions thing
    def drawSFC_3D(self,
                   sfc: MsgFlightCorridor):

        pass

