import numpy as np
import pyqtgraph.opengl as gl
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.tools.obstacles import RectangularObstacle


R = np.array([[0, 1, 0], 
              [1, 0, 0], 
              [0, 0, -1]])

class DrawMap:

    def __init__(self,
                 map: MsgWorldMap,
                 window: gl.GLViewWidget):
        

        self.window = window

        numDimensions = map.numDimensions_algorithm

        #with the window, let us start drawing up the obstacles
        fullMesh = np.array([], dtype=np.float32).reshape(0, 3, 3)
        fullMeshColors = np.array([], dtype=np.float32).reshape(0, 3, 4)



        self.obstaclePositionsList_unrotated = []
        self.obstaclePositionsList_rotated = []

        #iterates over all fo the obstacles in the list
        for obstacle in map.get_obstacles():

            #gets the obstacle positions
            obstaclePosition = obstacle.getTranslationWorld()
            self.obstaclePositionsList_unrotated.append(obstaclePosition)

            rotatedObstaclePosition = R @ obstaclePosition
            self.obstaclePositionsList_rotated.append(rotatedObstaclePosition)

            #gets the object vertices
            obstacleVertices_unrotated = obstacle.getVertices_obstacle_3D_list()

            obstacleVertices_rotated = []

            #rotates the positioning of the obstacle vertices so that down is up.
            for obstacleVertex in obstacleVertices_unrotated:

                rotatedVertex = R @ obstacleVertex
                obstacleVertices_rotated.append(rotatedVertex)

                

            obstacleMeshes, obstacleMeshColors = self.building_meshes_colors(vertices=obstacleVertices_rotated)

            #concatenates the full meshes and colors
            fullMesh = np.concatenate((fullMesh, obstacleMeshes), axis=0)
            fullMeshShape = np.shape(fullMesh)
            fullMeshColors = np.concatenate((fullMeshColors, obstacleMeshColors), axis=0)

        self.ground_mesh = gl.GLMeshItem(
            vertexes=fullMesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=fullMeshColors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        self.ground_mesh.setGLOptions('opaque')
        self.window.addItem(self.ground_mesh)


    #the function to get the meshes and mesh colors of the building
    def building_meshes_colors(self,
                               vertices: list[np.ndarray],
                               startBlock: bool = False):
        

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
        meshColors[0] = blue
        meshColors[1] = blue
        meshColors[2] = blue
        meshColors[3] = blue
        meshColors[4] = blue
        meshColors[5] = blue
        meshColors[6] = blue
        meshColors[7] = blue
        meshColors[8] = blue
        meshColors[9] = blue
        meshColors[10] = blue
        meshColors[11] = blue

        return meshes, meshColors
