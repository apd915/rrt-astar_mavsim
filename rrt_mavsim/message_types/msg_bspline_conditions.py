#this file implements the conditions class for the bspline pose

import numpy as np

class MsgBsplineConditions:

    #creates the initialization function
    def __init__(self,
                 position: np.ndarray = np.array([[0.0],[0.0],[0.0]]),
                 velocity: np.ndarray = np.array([[0.0],[0.0],[0.0]]),
                 acceleration: np.ndarray = np.array([[0.0],[0.0],[0.0]]))->None:
        #initializes the three things about the pose of a particular point
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        #puts them together into a list
        self.conditions = [self.position, self.velocity, self.acceleration]
        pass

    #updates the conditions list to reflect the position velocity and acceleration
    def updateConditions(self)->None:
        self.conditions = [self.position, self.velocity, self.acceleration]
        pass

    #function to get the conditions
    def getConditions(self)->np.ndarray:
        return self.conditions
    
    #function to set all the conditions
    def setConditions(self, 
                      position: np.ndarray,
                      velocity: np.ndarray,
                      acceleration: np.ndarray)->None:
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.updateConditions()
        pass
    
    #function to get and set position 
    def getPosition(self)->np.ndarray:
        return self.position
    
    def setPosition(self, position: np.ndarray)->None:
        self.position = position
        #updates the conditions list
        self.updateConditions()
        pass

    #functions to get and set velocity
    def getVelocity(self)->np.ndarray:
        return self.velocity
    
    def setVelocity(self, velocity: np.ndarray)->None:
        self.velocity = velocity
        self.updateConditions()
        pass

    #same for acceleration
    def getAcceleration(self)->np.ndarray:
        return self.acceleration
    
    def setAcceleration(self, accleration: np.ndarray)->None:
        self.acceleration = accleration
        self.updateConditions()
        pass