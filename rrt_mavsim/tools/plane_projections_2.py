
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

#maps a 3D vector in space to the plane. if the 3D vector does not 
#align with the plane already, it will be projected onto the plane and then 
#simulatenously mapped to the plane
def map_3D_to_2D(vec_3D: np.ndarray,
                 plane: MsgPlane):


    #gets the Q matrix from n_hat
    Q = plane.Q

    #gets the moore penrose pseudoinverse of Q
    Q_plus = np.linalg.pinv(a=Q)
    #gets the position in the 2D reference
    pos_2D = Q_plus @ (vec_3D - plane.origin_3D)

    #returns the 2D position
    return pos_2D

#creates the function to project a 3D position onto a plane (and stay in 3D)
def projectPosition_toPlane(vec_3D_init: np.ndarray,
                            plane: MsgPlane):

    #gets the origin of the plane
    p0 = plane.origin_3D
    n_hat = plane.n_hat


    #gets the vector from the plane's origin to the vec_3D_init
    vec_planeOrigin_to_3D_init = vec_3D_init - p0

    #gets the projection of this newfound vector onto n_hat
    dot_product_temp = np.dot(a=vec_planeOrigin_to_3D_init.flatten(), b=n_hat.flatten())
    n_hat_proj = (dot_product_temp/(np.linalg.norm(n_hat)**2))*n_hat

    #gets the vector form the plant origin to the projected location
    vec_planeOrigin_to_projectedPosition = vec_planeOrigin_to_3D_init - n_hat_proj

    #gets the 3D projected vector
    vec_3D_projected = vec_planeOrigin_to_projectedPosition + p0

    return vec_3D_projected



