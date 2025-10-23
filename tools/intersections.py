

import numpy as np
from scipy.optimize import linprog

#Arguments:
#A1: the A matrix for object 1
#b1: the b vector for object 1
#A2: the A matrix for object 2
#b2: the b vector for object 2
def intersectionOccurred(A1: np.ndarray,
                         b1: np.ndarray,
                         A2: np.ndarray,
                         b2: np.ndarray):
    
    #creates the new nex A and b matrices
    A_net = np.concatenate((A1, A2), axis=0)
    b_net = np.concatenate((b1, b2), axis=0)

    #creates a dummy variable that the linprog function will use for the feasibility here
    dummyVariable = np.zeros(A_net.shape[1])

    #uses linprog to find whether or not there exists a viable solution to the problem here

    result = linprog(dummyVariable, A_ub=A_net, b_ub=b_net, method='highs')

    intersectionOccurredTemp = result.success

    return intersectionOccurredTemp
