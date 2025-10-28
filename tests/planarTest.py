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

obs_1_Ab = worldMap.Ab_list[0]

obs_1_A = obs_1_Ab[0]
obs_1_b = obs_1_Ab[1]

obs_2_Ab = worldMap.Ab_list[-1]

obs_2_A = obs_2_Ab[0]
obs_2_b = obs_2_Ab[1]

#creates the normal vector for a plane
plane_n_vec = np.array([[1.0],[0.0],[0.0]])

n_plane_norm = plane_n_vec / np.linalg.norm(plane_n_vec)

#sets the position
p_0 = np.array([[200.0],[200.0],[-200.0]])


n_plane_norm_neg = n_plane_norm * -1.0


#gets the two b values
b_0 = n_plane_norm.T @ p_0
b_1 = n_plane_norm_neg.T @ p_0


#creates the new A matrix
A_1_new = np.concatenate((obs_1_A, n_plane_norm.T, n_plane_norm_neg.T), axis=0)
b_1_new = np.concatenate((obs_1_b, b_0, b_1), axis=0)


A_2_new = np.concatenate((obs_2_A, n_plane_norm.T, n_plane_norm_neg.T), axis=0)
b_2_new = np.concatenate((obs_2_b, b_0, b_1), axis=0)


c = np.ones(numDimensions)

result_1 = linprog(c, A_ub=A_1_new, b_ub=b_1_new, method='highs')
result_1_success = result_1.success


result_2 = linprog(c, A_ub=A_2_new, b_ub=b_2_new, method='highs')
result_2_success = result_2.success

#let's see of this 

potato = 0