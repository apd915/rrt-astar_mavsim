
#creates the tools for the alternative plane projection.
#this one gets a different basis 
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane



#according to the pixar algorithm, we obtain the basis for an orthonormal
#set, given the first vector from that set (given n1, find n2 and n3
#where n2 and n3 span the working plane being defined)
def getNormalBasis(n_hat: np.ndarray):

    n_x = n_hat.item(0)
    n_y = n_hat.item(1)
    n_z = n_hat.item(2)
    #gets the sign of the z component of the n_hat vector
    nz_sign = np.copysign(1.0, n_z)
    
    #gets the a term
    a = -1.0/(nz_sign + n_z)
    #gets the b term
    b = n_x * n_y * a


    #constructs the u1 (output 1 vector)
    u1 = np.array([[1.0 + nz_sign*(n_x**2)*a],
                   [nz_sign * b],
                   [-nz_sign*n_x]])
    
    #same with the u2 vector
    u2 = np.array([[b],
                   [nz_sign + (n_y**2)*a],
                   [-n_y]])

    Q = np.concatenate((u1, u2), axis=1)

    testPoint = 0


    return Q


#maps a 2D vector to a 3D vector in space
#whether it be a position, a velocity, or an acceleration
#it can be any number of vectors of shape (2,N)
#returns a vector of shape (3,N)
def map_2D_to_3D(vec_2D: np.ndarray,
                 plane: MsgPlane):

    #gets the number of vectors in vec_3D
    numVectors = np.shape(vec_2D)[1]
    
    #gets the basis vectors
    Q = plane.Q

    #gets the origin
    origin_3D = plane.origin_3D
    #tiles the origin
    origin_3D_tiled = np.repeat(origin_3D, numVectors, axis=1)

    #gets the vec_3D
    vec_3D = origin_3D_tiled + Q @ vec_2D
    
    return vec_3D



