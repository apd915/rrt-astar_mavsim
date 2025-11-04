import numpy as np


#sets the e1, e2, and e3 vectors
e1 = np.array([[1.0],[0.0],[0.0]])
e2 = np.array([[0.0],[1.0],[0.0]])
e3 = np.array([[0.0],[0.0],[1.0]])


#sets the vector shape
vector_shape = (3,1)

#sets the epsilon (the point at which, if the cross product magnitude is less than epsilon
#then we manually set the u vector)
epsilon = 1e-6

