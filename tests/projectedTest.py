import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from planners.rrt_sfc_bspline import projectPosition_toPlane




#sets the offset
p0 = np.array([[0.0],[0.0],[1.0]])

n_hat = np.array([[0.0],[1.0],[1.0]])
n_hat = n_hat/np.linalg.norm(n_hat)

pos_3D = np.array([[10.0],[10.0],[10.0]])



projectedVector = projectPosition_toPlane(pos_3D=pos_3D,
                                  p_0=p0,
                                  n_hat=n_hat)

print(projectedVector)

potato = 0
