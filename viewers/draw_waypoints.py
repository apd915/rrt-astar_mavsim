#defines the function to draw the waypoints 
import numpy as np
import pyqtgraph.opengl as gl
from message_types.msg_waypoints import MsgWaypoints_SFC
from message_types.msg_flight_corridors import MsgFlightCorridor


R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
red = np.array([[204, 0, 0],
                [204, 0, 0]])/255.

purple = np.array([[170, 0, 255],
                   [170, 0, 255]])/255


class DrawWaypoints:


    def __init__(self,
                 waypoints: MsgWaypoints_SFC,
                 window: gl.GLViewWidget,
                 lineColor: np.ndarray = red):


        #gets the dimension
        numDimensions = waypoints.numDimensions

        #gets the list of SFCs
        flightCorridor_list = waypoints.getAllFlightCorridors()


        for flightCorridor in flightCorridor_list:

            if numDimensions == 2:
                self.drawSFC_2D(flightCorridor=flightCorridor,
                                color=lineColor,
                                )
                
            elif numDimensions == 3:

                self.drawSFC_3D(sfc=flightCorridor)


    #defines the draw 2 dimensions thing
    def drawSFC_2D(self,
                   flightCorridor: MsgFlightCorridor,
                   color: np.ndarray,
                   lineWidth: float,
                   window: gl.GLViewWidget):
        
        #gets the sfc from the flight corridor
        sfc_temp = flightCorridor.generateSFC()
        


        pass

    #defines the draw 3 dimensions thing
    def drawSFC_3D(self,
                   sfc: MsgFlightCorridor):

        pass

