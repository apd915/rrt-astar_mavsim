import numpy as np
import pyqtgraph.opengl as gl


points = [np.array([[-1.0],[-1.0],[-1.0]]),
          np.array([[1.0],[-1.0],[-1.0]]),
          np.array([[1.0],[1.0],[-1.0]]),
          np.array([[-1.0],[1.0],[-1.0]]),
          np.array([[-1.0],[-1.0],[1.0]]),
          np.array([[1.0],[-1.0],[1.0]]),
          np.array([[1.0],[1.0],[1.0]]),
          np.array([[-1.0],[1.0],[1.0]])]



class DrawEndMarker:

    def __init__(self,
                 position: np.ndarray,
                 scale: float,
                 window: gl.GLViewWidget):

        #saves the window
        self.window = window
        self.position = position
        self.scale = scale


        outputPoints = []

        for point in points:
            scaledPoint = point*self.scale
            shiftedPoint = scaledPoint + position

            outputPoints.append(shiftedPoint)

        #gets the meshes and mesh colors
        self.meshes, self.meshColors = self.building_vert_face(vertices=outputPoints)

        self.markerMesh = gl.GLMeshItem(vertexes=self.meshes,
                                        vertexColors=self.meshColors,
                                        drawEdges=True,
                                        smooth=False,
                                        computeNormals=False)
        self.markerMesh.setGLOptions('translucent')
        self.window.addItem(self.markerMesh)




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
        meshColors[0] = blue
        meshColors[1] = blue
        meshColors[2] = blue
        meshColors[3] = blue
        meshColors[4] = blue
        meshColors[5] = blue
        meshColors[6] = blue
        meshColors[7] = blue
        meshColors[8] = red
        meshColors[9] = red
        meshColors[10] = red
        meshColors[11] = red

        return meshes, meshColors