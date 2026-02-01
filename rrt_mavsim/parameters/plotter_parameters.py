import numpy as np



#rotation from NED frame to altitude frame
R_NED_to_Altitude = np.array([[0, 1, 0], 
                              [1, 0, 0], 
                              [0, 0, -1]])
