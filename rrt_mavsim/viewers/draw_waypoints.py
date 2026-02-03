#defines the function to draw the waypoints 
import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
import rrt_mavsim.parameters.display_parameters as DISPLAY
from rrt_mavsim.message_types.msg_plane import MsgPlane


R = np.array([[0, 1, 0], 
              [1, 0, 0], 
              [0, 0, -1]])



red = (1.0, 0.0, 0.0, 1.0)
purple = (170/255, 0, 1.0, 1.0)

class DrawWaypoints:


    def __init__(self,
                 waypoints: MsgWaypoints_SFC,
                 window: gl.GLViewWidget,
                 lineColor: tuple = red,
                 plane: MsgPlane = None):


        #gets the dimension
        numDimensions = waypoints.numDimensions

        #gets the list of SFCs
        flightCorridor_list = waypoints.getAllFlightCorridors()

        self.plane = plane

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
        normalsList, vertices_list = flightCorridor.getNormalsVertices_3D(plane=self.plane)

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

