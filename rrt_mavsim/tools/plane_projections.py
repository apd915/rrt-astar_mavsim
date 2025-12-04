import numpy as np

import rrt_mavsim.parameters.projection_parameters as PROJ
from rrt_mavsim.message_types.msg_plane import MsgPlane

#defines the function to project a position onto a plane
def projectPosition_toPlane(pos_3D: np.ndarray, #the 3D position of the point (the point not on the plane)
                            p_0: np.ndarray, #the 3D position of the origin on the plane (The zero position on the plane)
                            n_hat: np.ndarray): #the unit vector of the plane 
    
    #gets the vector from p0 to pos_3D
    vec_p0_to_point = pos_3D - p_0

    dotProduct = np.dot(a=vec_p0_to_point.flatten(), b=n_hat.flatten())
    #now that this is relative to the origin, gets the projection onto the n_hat
    n_hat_proj = (dotProduct/(np.linalg.norm(n_hat)**2))*n_hat
    #gets the projection onto the plane, which is the vector minus the n_hat projection
    vec_proj = vec_p0_to_point - n_hat_proj
    #returns the projected vector
    return vec_proj


#defines the function to map from 2D to 3D
#pos_2D: the position given in the frame of the plane (2x1)
#Q: the frame of the plane (3x2)
#p0: the offset of the origin of the plane from the origin of the 3d map
def map_2D_to_3D(vec_2D: np.ndarray,
                 n_hat: np.ndarray,
                 p0: np.ndarray):

    #gets the Q matrix from n_hat
    Q = getPlaneBasis(n_hat=n_hat)

    #multiplies it out to get the 3D position
    pos_3D = p0 + Q @ vec_2D

    #returns the 3D position
    return pos_3D

#same thing, but I'm lazy and I want to just pass in the plane message
def map_2D_to_3D_planeMsg(vec_2D: np.ndarray,
                          plane_msg: MsgPlane):
    
    Q = getPlaneBasis(n_hat=plane_msg.n_hat)

    pos_3D = plane_msg.origin_3D + Q @ vec_2D

    return pos_3D

#define sthe function to map from a 3D position (on the plane)
#to the Plane's 2D position coordinate system
def map_3D_to_2D(vec_3D: np.ndarray,
                 n_hat: np.ndarray,
                 p0: np.ndarray):
    
    #gets the Q matrix from n_hat
    Q = getPlaneBasis(n_hat=n_hat)

    #gets the moore penrose pseudoinverse of Q
    Q_plus = np.linalg.pinv(a=Q)
    #gets the position in the 2D reference
    pos_2D = Q_plus @ (vec_3D - p0)

    #returns the 2D position
    return pos_2D


def map_3D_to_2D_planeMsg(vec_3D: np.ndarray,
                          plane_msg: MsgPlane):

    #gets the Q matrix from n_hat
    Q = getPlaneBasis(n_hat=plane_msg.n_hat)

    #gets the moore penrose pseudoinverse of Q
    Q_plus = np.linalg.pinv(a=Q)
    #gets the position in the 2D reference
    pos_2D = Q_plus @ (vec_3D - plane_msg.origin_3D)

    #returns the 2D position
    return pos_2D


    


def getPlaneBasis(n_hat: np.ndarray):

    n_hat = n_hat / np.linalg.norm(n_hat)

    #gets the cross product between e3 and n hat
    u_candidate = (np.cross(n_hat.flatten(), PROJ.e3.flatten())).reshape(PROJ.vector_shape)

    #gets the u candidate norm
    u_candidate_magnitude = np.linalg.norm(u_candidate)

    #edge case we are super close to parallel for n_hat and e3, we manually set the u vector
    #case n_hat is in the 
    if u_candidate_magnitude < PROJ.epsilon:
        #always sets the u1 to the forward x direction
        u1 = PROJ.e1

        #if this is the case, the vector is mostly in either the positive or negative 
        #z direction, so we get that value and see its sign
        z_component = n_hat[2,0]
        #gets the sign of the z component
        z_sign = np.sign(z_component)

        #if the sign is positive, we go with a positive e2 for u2
        if z_sign > 0:
            u2 = PROJ.e2
        #if the sign is negative, we go with a negative e2 for u2
        else:
            u2 = -PROJ.e2

    
    #otherwise we get it with the cross product
    else:

        #sets u1
        u1 = u_candidate / u_candidate_magnitude

        u2 = np.cross(n_hat.flatten(), u1.flatten()).reshape(PROJ.vector_shape)
        #normalizes it just in case
        u2 = u2 / np.linalg.norm(u2)

    #creates the Q vector
    Q = np.concatenate((u1, u2), axis=1)

    return Q
