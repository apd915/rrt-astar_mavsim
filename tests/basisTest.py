import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np


from rrt_mavsim.message_types.msg_world_map import getPlaneBasis



#sets the n_hat
n_hat = np.array([[0.0],[0.0],[1.0]])


Q = getPlaneBasis(n_hat=n_hat)

potato = 0