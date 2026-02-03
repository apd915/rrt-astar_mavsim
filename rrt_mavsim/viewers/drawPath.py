import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
import rrt_mavsim.parameters.display_parameters as DISPLAY
from rrt_mavsim.message_types.msg_plane import MsgPlane
from bsplinegenerator.bsplines import BsplineEvaluation

#draws the path in the viewer

R = np.array([[0, 1, 0], 
              [1, 0, 0], 
              [0, 0, -1]])


#defines the class to draw the control points
class DrawPath:

    def __init__(self,
                 controlPoints: np.ndarray = None,
                 window: gl.GLViewWidget = None,
                 lineColor: np.ndarray = None,
                 lineWidth: float = None,
                 pointWidth: float = None):
        
