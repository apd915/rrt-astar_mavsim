
#draws the map of the obstacles

import numpy as np
import pyqtgraph.opengl as gl
from message_types.msg_world_map import MsgWorldMap


class DrawMap:

    def __init__(self,
                 map: MsgWorldMap,
                 window: gl.GLViewWidget,
                 minCutoffAltitude: float = None):
        
        self.window = window


        #gets the number of dimensions
        numDimensions = map.numDimensions


        #with the window, let us start drawing up the obstacles
        fullMesh = np.array([], dtype=np.float32).reshape(0, 3, 3)
        fullMeshColors = np.array([], dtype=np.float32).reshape(0, 3, 4)


        #if this is a second dimension thing
        if numDimensions == 2:
            #gets the vertices list
            
            pass

        elif numDimensions == 3:

            pass


    def building_vert_face(self,
                           vertices: list[np.ndarray]):
        

        pts = []


        for vertex in vertices:
            flattenedVertex = vertex.flatten()
            pts.append(flattenedVertex)
        
        #creates the meshes list
        meshes = np.array([[pts[0], pts[1], pts[2]],
                           [pts[0], pts[2], pts[3]],
                           [pts[0], pts[5], pts[4]],
                           [pts[0], pts[1], pts[5]],
                           [pts[1], pts[6], pts[5]],
                           [pts[1], pts[2], pts[6]],
                           [pts[2], pts[6], pts[7]],
                           [pts[2], pts[7], pts[3]],
                           [pts[3], pts[4], pts[7]],
                           [pts[3], pts[0], pts[4]],
                           [pts[4], pts[5], pts[6]],
                           [pts[4], pts[6], pts[7]]])
        
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = 'g'
        meshColors[1] = 'g'
        meshColors[2] = 'g'
        meshColors[3] = 'g'
        meshColors[4] = 'g'
        meshColors[5] = 'g'
        meshColors[6] = 'g'
        meshColors[7] = 'g'
        meshColors[8] = 'y'
        meshColors[9] = 'y'
        meshColors[10] = 'y'
        meshColors[11] = 'y'

        return meshes, meshColors