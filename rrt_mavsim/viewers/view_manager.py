#creates the class to view the Obstacles and the Fligth path and everything

import pyqtgraph as pg
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.viewers.planner_viewer import PlannerViewer
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
import numpy as np


class ViewManager:

    def __init__(self,
                 mav: bool = False,
                 planningFlag: bool = True):
        
        self.mav_flag = mav
        self.planningFlag = planningFlag

        #creates a Q widget application
        self.app = pg.QtWidgets.QApplication([]) 


        if self.planningFlag:

            self.planner_viewer = PlannerViewer(app=self.app)


    def update_planning_tree(self,
                             waypoints: MsgWaypoints_SFC,
                             waypoints_not_smooth: MsgWaypoints_SFC,
                             tree: MsgWaypoints_SFC,
                             world_map: MsgWorldMap,
                             optimizedControlPoints: np.ndarray):
        

        self.planner_viewer.draw_tree_and_map(worldMap=world_map,
                                              tree=tree,
                                              waypoints=waypoints,
                                              waypoints_not_smooth=waypoints_not_smooth,
                                              optimizedControlPoints=optimizedControlPoints)