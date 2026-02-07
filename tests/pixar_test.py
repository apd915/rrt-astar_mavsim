import numpy as np
from rrt_mavsim.tools.plane_projections_2 import getNormalBasis
from rrt_mavsim.message_types.msg_plane import MsgPlane


n_hat = np.array([[0.0],[-1.0],[0.0]])
origin = np.array([[0.0],[0.0],[0.0]])

#creates the plane message
plane = MsgPlane(n_hat=n_hat, origin_3D=origin)


testPoint =0
