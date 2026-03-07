# creates the class to view the Obstacles and the Fligth path and everything

import pyqtgraph as pg
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.viewers.planner_viewer import PlannerViewer
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_plane import MsgPlane
import numpy as np


class ViewManager:
    def __init__(self, mav: bool = False, planningFlag: bool = True):
        self.mav_flag = mav
        self.planningFlag = planningFlag

        # creates a Q widget application
        self.app = pg.QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = pg.QtWidgets.QApplication([])

        if self.planningFlag:
            self.planner_viewer = PlannerViewer(app=self.app)

    def update_planning_tree(
        self,
        waypoints_not_smooth: MsgWaypoints_SFC,
        waypoints_smooth: MsgWaypoints_SFC,
        degree: int,
        numdimensions: int,
        R_ned_to_alt: np.ndarray,
        tree: MsgWaypoints_SFC,
        world_map: MsgWorldMap,
        controlPoints_list: list[np.ndarray],
        plane: MsgPlane = None
    ):
        self.planner_viewer.draw_tree_and_map(
            worldMap=world_map,
            tree=tree,
            dimension=numdimensions,
            degree=degree,
            R_ned_to_alt=R_ned_to_alt,
            waypoints_not_smooth=waypoints_not_smooth,
            waypoints_smooth=waypoints_smooth,
            controlPoints_list=controlPoints_list,
            plane=plane
        )

    def drawMap(self,
                world_map: MsgWorldMap):

        self.planner_viewer.draw_map(worldMap=world_map)

