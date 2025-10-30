import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
from message_types.msg_world_map import MsgWorldMap
from scipy.optimize import linprog

numDimensions = 3


worldMap = MsgWorldMap(obstacleFieldType='rectangular',
                       numDimensions=numDimensions)

obs_1_Ab = worldMap.Ab_3D_list[0]

obs_1_A = obs_1_Ab[0]
obs_1_b = obs_1_Ab[1]

obs_2_Ab = worldMap.Ab_3D_list[-1]

obs_2_A = obs_2_Ab[0]
obs_2_b = obs_2_Ab[1]

#creates the normal vector for a plane
plane_n_vec = np.array([[1.0],[0.0],[0.0]])

n_plane_norm = plane_n_vec / np.linalg.norm(plane_n_vec)

#sets the position
p_0 = np.array([[200.0],[200.0],[-200.0]])

p_0_norm = p_0 / np.linalg.norm(p_0)

#gets the SFC of the N plane norm
U, S, Vt = np.linalg.svd(p_0_norm.T)

Vt_0 = Vt[:,0:1]
Vt_1 = Vt[:,1:2]
Vt_2 = Vt[:,2:3]

crossProduct = np.cross(Vt_1.flatten(), Vt_2.flatten())


#creates the U matrix
Proj = Vt[:,0:2]


#creates the new A and b vectors
A_1_new = obs_1_A @ Proj

b_1_new = obs_1_b - obs_1_A @ p_0

A_2_new = obs_2_A @ Proj

b_2_new = obs_2_b - obs_2_A @ p_0


#sees if these are feasible
c = np.ones(numDimensions-1)

result_1 = linprog(c, A_ub=A_1_new, b_ub=b_1_new, method='highs')

result_1_status = result_1.success


result_2 = linprog(c, A_ub=A_2_new, b_ub=b_2_new, method='highs')

result_2_status = result_2.success

#let's see of this 

potato = 0