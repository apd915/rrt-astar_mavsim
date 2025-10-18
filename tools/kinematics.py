#this file implements helper matrices for kinematic rotations and so forth

import numpy as np
from numpy import cos as c
from numpy import sin as s


#rotates from body frame velocities to world frame velocities
def R_body_inertial_velocities(phi: float, theta: float, psi: float):

    R = np.array([[c(theta)*c(psi), s(phi)*s(theta)*c(psi) - c(phi)*s(psi), c(phi)*s(theta)*c(psi) + s(phi)*s(psi)],
                  [c(theta)*s(psi), s(phi)*s(theta)*s(psi) + c(phi)*c(psi), c(phi)*s(theta)*s(psi) - s(phi)*c(psi)],
                  [-s(theta), s(phi)*c(theta), c(phi)*c(theta)]])
    
    return R