import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np

#from the
from tools.getEulerUnit import getRotFromUnitVec



#creates the main vector
vec1 = np.array([[0],[0],[1]])
vec1_norm = vec1 / np.linalg.norm(vec1)

getRotFromUnitVec(unitVec=vec1_norm)

