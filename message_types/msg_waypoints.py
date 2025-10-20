import numpy as np
from message_types.msg_flight_corridors import MsgFlightCorridor

class MsgWaypoints_SFC:

    #creates the initialization function
    def __init__(self,
                 numDimensions: int):
        
        #creates list of numpy arrays containing positional information
        self.positions = []
        #list of parent indices for each position
        self.parents = []
        #list of the costs for each position
        self.costs = []
        #list for the flight corridors
        self.flightCorridors = []
        #list for whether or not it connects to the goal
        self.connectsToGoal = []
        #number of positions
        self.numPositions = 0

        self.numDimensions = numDimensions

    #function to add a waypoint to the waypoints list\
    def add(self,
            position: np.ndarray = None,
            parent: int = None,
            cost: float = None,
            connectsToGoal: bool = None):
        
        if position is not None:
            self.positions.append(position)
            #increments the number of positions in the tree
            self.numPositions += 1
        if parent is not None:
            self.parents.append(parent)
        if cost is not None:
            self.costs.append(cost)
        if connectsToGoal is not None:
            self.connectsToGoal.append(connectsToGoal)

    #creates the addition function
    def addSFC(self,
               sfc: MsgFlightCorridor):
        #security to make sure we append 
        if sfc is not None:
            #gets the number of dimensions for the sfc
            numDimensions_inputSFC = sfc.getNumDimensions()
            if numDimensions_inputSFC != self.numDimensions:
                raise ValueError("Number of Dimensions is inconsistent")
            self.flightCorridors.append(sfc)
        else:
            raise TypeError("none Type Detected for SFC")


    #gets the different arrays
    def getAllPositions(self):
        return self.positions
        
    def getAllFlightCorridors(self):
        return self.flightCorridors
    

    def getPosition(self,
                    index: int):
        return (self.positions)[index]
    def getCost(self,
                index: int):
         return (self.costs)[index]
    def getParent(self,
                  index: int):
        return (self.parents)[index]
    def getFlightCorridor(self,
                          index: int):
        return (self.flightCorridors)[index]