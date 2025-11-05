import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.tools.obstacles import RectangularObstacle


class DrawMap:

    def __init__(self,
                 map: MsgWorldMap,
                 window: gl.GLViewWidget):
        

        self.window = window

        numDimensions = map.numDimensions_algorithm

        #with the window, let us start drawing up the obstacles
        fullMesh = np.array([], dtype=np.float32).reshape(0, 3, 3)
        fullMeshColors = np.array([], dtype=np.float32).reshape(0, 3, 4)


        #iterates over all fo the obstacles in the list
        for obstacle in map.get_obstacles():

            #gets the object vertices
            obstacleVertices = obstacle.getVertices_obstacle_3D_list()

            obstacleMeshes, obstacleMeshColors = self.building_meshes_colors(vertices=obstacleVertices)

            #concatenates the full meshes and colors
            fullMesh = np.concatenate((fullMesh, obstacleMeshes), axis=0)
            fullMeshColors = np.concatenate((fullMeshColors, obstacleMeshColors), axis=0)

        self.ground_mesh = gl.GLMeshItem(
            vertexes=fullMesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=fullMeshColors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        self.ground_mesh.setGLOptions('translucent')
        self.window.addItem(self.ground_mesh)


    #the function to get the meshes and mesh colors of the building
    def building_meshes_colors(self,
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