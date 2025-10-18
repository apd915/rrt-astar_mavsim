#defines the class to implement the seperating axis theorem to determine overlap of 
#convex polygons

import numpy as np


class seperatingAxis:

    #creates the initialization function
    def __init__(self):

        pass


    #creates the function to determine if there is overlap of the convex hull of two sets
    #of vertices
    def hullsIntersect(vertices_1: list[np.ndarray],
                       vertices_2: list[np.ndarray]):