

import numpy as np
import pyqtgraph.opengl as gl
from message_types.msg_world_map import MsgWorldMap
from message_types.msg_waypoints import MsgWaypoints_SFC

class PlannerViewer:

    def __init__(self):

        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('RRT Tree Viewer')
        self.window.setGeometry(500, 0, 500, 500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        center = self.window.cameraPosition()
        center.setX(1000)
        center.setY(1000)
        center.setZ(0)
        self.window.setCameraPosition(pos=center, 
                                      distance=self.scale, 
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
                          optimizedControlPoints: np.ndarray = None):
        