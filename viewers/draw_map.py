
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
            obstacleList_2D = map.get_2D_obstacles()

            for northList in obstacleList_2D:
                for object in northList:
                    
                    #gets the vertices list of the object
                    objectVertices = object.getVertices_building_2D_list()

                    currentMeshes, currentColors = self.building_vert_face(vertices=objectVertices)

                    #appends to the full mesh and colors list
                    fullMesh = np.concatenate((fullMesh, currentMeshes), axis=0)
                    fullMeshColors = np.concatenate((fullMeshColors, currentColors), axis=0)
                    
        elif numDimensions == 3:
            #gets the obstacles list
            obstacleList_3D = map.get_3D_obstacles()

            for northList in obstacleList_3D:
                for eastList in northList:
                    for object in eastList:

                        objectVertices = object.getVertices_obstacle_3D_list()

                        currentMeshes, currentColors = self.building_vert_face(vertices=objectVertices)
    
                        #appends to the full mesh and colors list
                        fullMesh = np.concatenate((fullMesh, currentMeshes), axis=0)
                        fullMeshColors = np.concatenate((fullMeshColors, currentColors), axis=0)


        self.ground_mesh = gl.GLMeshItem(
            vertexes=fullMesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=fullMeshColors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        self.ground_mesh.setGLOptions('translucent')
        self.window.addItem(self.ground_mesh)

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

        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])

        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = green
        meshColors[1] = green
        meshColors[2] = green
        meshColors[3] = green
        meshColors[4] = green
        meshColors[5] = green
        meshColors[6] = green
        meshColors[7] = green
        meshColors[8] = yellow
        meshColors[9] = yellow
        meshColors[10] = yellow
        meshColors[11] = yellow

        return meshes, meshColors