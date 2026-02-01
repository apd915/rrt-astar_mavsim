

import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.viewers.draw_map import DrawMap
import pyqtgraph as pg
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
from rrt_mavsim.viewers.draw_waypoints import DrawWaypoints
from rrt_mavsim.viewers.drawEndPoints import DrawEndMarker
from rrt_mavsim.message_types.msg_plane import MsgPlane


class PlannerViewer:

    def __init__(self,
                 app: pg.QtWidgets.QApplication):
        
        self.app = app
        
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('RRT Tree Viewer')
        self.window.setGeometry(500, 0, 500, 500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(PLAN.scale/20, PLAN.scale/20, PLAN.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        center = self.window.cameraPosition()
        center.setX(1000)
        center.setY(1000)
        center.setZ(0)
        self.window.setCameraPosition(pos=center, 
                                      distance=PLAN.scale, 
                                      elevation=50, 
                                      azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        #self.window.setBackgroundColor('w')  # set background color to black
        # self.window.resize(*(4000, 4000))  # not sure how to resize window
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])


    def draw_tree_and_map(self,
                          worldMap: MsgWorldMap,
                          tree: MsgWaypoints_SFC,
                          waypoints: MsgWaypoints_SFC,
                          waypoints_not_smooth: MsgWaypoints_SFC,
                          optimizedControlPoints: np.ndarray = None,
                          plane: MsgPlane = None):
        
        DrawMap(map=worldMap,
                window=self.window)
        
        #draws the waypoints out
        DrawWaypoints(waypoints=waypoints,
                      window=self.window,
                      plane=plane)
        

    def draw_map(self,
                 worldMap: MsgWorldMap):
        DrawMap(map=worldMap,
                window=self.window)
