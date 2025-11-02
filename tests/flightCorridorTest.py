import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from message_types.msg_flight_corridors import MsgFlightCorridor



flightCorridor_2D = MsgFlightCorridor(numDimensions=2,
                                      primaryPosition=np.array([[10.0],[10.0]]),
                                      secondaryPosition=np.array([[11.0],[11.0]]))

flightCorridor_3D = MsgFlightCorridor(numDimensions=3,
                                      primaryPosition=np.array([[10.0],[10.0],[10.0]]),
                                      secondaryPosition=np.array([[11.0],[11.0],[11.0]]))


fat = 0