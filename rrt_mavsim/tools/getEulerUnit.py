#functions to get the pitch and yaw given the unit vector input
import numpy as np
from rrt_mavsim.tools.rotations import euler_to_rotation

fudgeFactor = 0.97

def getRotFromUnitVec(unitVec: np.ndarray):

    #checks for the singularity where pitch magnitude is 90 degrees
    north = unitVec.item(0)
    east = unitVec.item(1)
    down = unitVec.item(2)

    #two cases to avoid the singularity
    if down > fudgeFactor:
        pitch = -np.pi
        yaw = 0.0

    elif down < -fudgeFactor:

        pitch = np.pi
        yaw = 0.0
    
    else:
        
        #gets the bottom plane projection
        planeProjection = np.array([[north],[east],[0.0]])
        #gets the length of the plane projection
        planeProjection_length = np.linalg.norm(planeProjection)
        #sets the pitch angle
        pitch = -np.arctan2(down, planeProjection_length)
        
        yaw = np.arctan2(east, north)


    R = euler_to_rotation(phi=0.0,
                          theta=pitch,
                          psi=yaw)
    
    return R