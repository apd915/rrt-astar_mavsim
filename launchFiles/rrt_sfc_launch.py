#creates the launch file for the B-Spline rrt SFC launch

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from message_types.msg_world_map import MsgWorldMap


worldMap = MsgWorldMap(obstacleFieldType='rectangular',
                       numDimensions = 2,
                       fieldWidth=2000.0,
                       obstacleWidthRatio=0.5,
                       obstacleWidth_sigma=1.0,
                       altitude=100.0,
                       numBlocks=4)


potato = 0

samwise = 0


tomato = 0
