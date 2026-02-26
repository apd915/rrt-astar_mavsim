import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import projectPosition_toPlane



n_hat = np.array([[1.0],[-1.0],[0.0]])
n_hat = n_hat / np.linalg.norm(n_hat)


#creates the plane
plane = MsgPlane(n_hat=n_hat,
                 origin_3D=np.array([[10.0],[0.0],[0.0]]))

position_1_proj = projectPosition_toPlane(vec_3D_init=np.array([[20.0],[0.0],[0.0]]),
                                          plane=plane)



testPoint = 0
