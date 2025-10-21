#creates the class to view the Obstacles and the Fligth path and everything

import pyqtgraph as pg
from message_types.msg_waypoints import MsgWaypoints_SFC


class ViewManager:

    def __init__(self,
                 mav: bool = False):
        
        self.mav_flag = mav

    def update_planning_tree(self,
                             waypoints: MsgWaypoints_SFC):
        

        