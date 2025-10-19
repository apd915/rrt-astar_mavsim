#creates parameters to help with the bspline


import numpy as np
from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from eVTOL_BSplines.message_types.msg_control_points import MSG_Control_Points
from bsplinegenerator.bsplines import BsplineEvaluation
from message_types.msg_bspline_conditions import MsgBsplineConditions
from message_types.msg_waypoints import MsgWaypoints_SFC
import time
import copy



degree = 3

class BsplineParameters:

    def __init__(self):
        #instantiates the class of the static flight path
        self.pathGenerator = staticFlightPath()

        #initializes the other things, and sets them to none initially.
        self.rho = None
        self.numDimensions = None
        self.M = None
        self.d = None
        self.start_conditions = None
        self.end_conditions = None
        self.BSpline = None
        pass

    

    #updates the start and end conditions of the bspline parameter
    def update(self,
               rho: np.ndarray, #mixing array which mixes derivatives
               numDimensions: int, #the dimensionality of the problem here
               d: int, #the degree of the bspline
               M: int, #the number of intervals of interest for the problem
               start_conditions: MsgBsplineConditions, #the initial conditions
               end_conditions: MsgBsplineConditions): #the final conditions
        
        #saves all of these parameters

        self.rho = rho
        self.numDimensions = numDimensions
        self.M = M
        self.d = d
        self.start_conditions = start_conditions
        self.end_conditions = end_conditions


        #gets the list of conditions
        startConditionsArray = start_conditions.getConditions()
        endConditionsArray = end_conditions.getConditions()

        #calls the function to get the control points
        self.controlPoints_array = (self.pathGenerator).getControlPoints(initialConditions=startConditionsArray,
                                                                    finalConditions=endConditionsArray,
                                                                    rho=rho,
                                                                    numDimensions=numDimensions,
                                                                    d=d,
                                                                    M=M)

        #creates the bspline evaluation
        self.BSpline = BsplineEvaluation(control_points=self.controlPoints_array,
                                        order=degree,
                                        start_time=0.0)


        
        
    #creates the function to compute the sampled points of the bspline
    def compute_sampled_points(self,
                               numSamplePointsPerInterval: int): #gets a certain number of data points per interval
        #gets all the samples
        self.samples, timeData = self.BSpline.get_spline_data(num_data_points_per_interval=numSamplePointsPerInterval)

        return self.samples
    
    #creates the wrapper function to plot the bspline out
    def plot_bspline(self,
                     numSamplePointsPerInterval: int=100):
        (self.BSpline).plot_spline(num_data_points_per_interval=numSamplePointsPerInterval)

    #defines the function to get the control points for a section
    def getControlPoints_array(self):
        return self.controlPoints_array



    #defines the function to get the minvo control points
    def getMinvoControlPointsArray(self):
        #gets  and returns the minvoControl points
        minvoControlPoints = self.BSpline.get_minvo_control_points()
        return minvoControlPoints
    
    #defines the function to get the minvo control points as a list of a list of 2d arrays
    def getMinvoControlPointsList(self):
        
        #gets the minco points
        minvoControlPoints = self.BSpline.get_minvo_control_points()

        #creates the minvo main list
        minvoTotalList = []
        #iterates over all the intervals of interest
        for i in range(self.M):
            #creates the minvo section list
            minvoSectionList = []
            #iterates over the degree plus 1
            for j in range(self.d + 1):

                #gets the currentIndex, which jumps over M's
                currentIndex = i*(self.d + 1) + j

                #gets the temp position array
                tempPosition = minvoControlPoints[0:2, currentIndex:(currentIndex+1)]
                
                #appends to the minvo section list
                minvoSectionList.append(tempPosition)

            #appends the minvo section list to the minvo total list
            minvoTotalList.append(minvoSectionList)

        #returns the minvo total list
        return minvoTotalList
    
    #defines the function to get the minvo control points, but all as one big list
    def getMinvoControlPointsSingleList(self):
        #gets the minvo points
        minvoControlPoints = self.BSpline.get_minvo_control_points()

        #creates the minvo main list
        minvoTotalList = []
        #gets the shape of the minvo control points
        minvoControlPointsShape = np.shape(minvoControlPoints)
        #gets the number of points, which is the second item on the list
        numPoints = minvoControlPointsShape[1]

        for currentIndex in range(numPoints):
            #gets the temp position
            tempPosition = minvoControlPoints[0:2, currentIndex:(currentIndex+1)]
            #appends the temp position to the list
            minvoTotalList.append(tempPosition)


        return minvoTotalList
    
    #defines the function to get the the joint list
    def getMinvoControlPointsJointLists(self):
        #gets the minvo control points here
        minvoControlPoints = self.BSpline.get_minvo_control_points()

        #gets the shape of the minvo control points
        minvoControlPointsShape = np.shape(minvoControlPoints)
        #gets the number of points, which is the second item on the list
        numPoints = minvoControlPointsShape[1]

        #creates the minvo Total List
        minvoTotalList = []
        #creates the minvo list of list
        minvoListList = []
        #initializes the temp list
        tempList = []
        for currentIndex in range(numPoints):
            #gets the temp point
            tempPoint = minvoControlPoints[0:2, currentIndex:(currentIndex+1)]

            #appends the temp point to the total list
            minvoTotalList.append(tempPoint)
            
            #appends to the temp list
            tempList.append(tempPoint)

            #if the current index is at d, then we append to the temp list to the
            #List of Lists, and then we reset the temp list
            if (currentIndex % (self.d + 1)) == self.d:
                #appends the temp list
                minvoListList.append(tempList)
                #resets the temp list
                tempList = []
        
        #returns the total list and the list list
        return minvoTotalList, minvoListList

    #creates the function to get the BSpline
    def getBSpline(self):
        return self.BSpline
    
    #creates the function to get the spline length
    def getBSplineLength(self, numSamplePointsPerInterval: int = 100):
        #gets the length
        length = (self.BSpline).get_arc_length(resolution=numSamplePointsPerInterval)

        #returns the length of the thing
        return length
        

#defines the helper function to split up the control points array into a useable list
def getListFromArray(inputArray: np.ndarray)->list[np.ndarray]:

    #gets the shape of the array
    arrayShape = np.shape(inputArray)
    
    #gets the dimension
    dimension = arrayShape[0]

    #gets the number of elements
    numElements = arrayShape[1]


    outputList = []

    #iterates over all the elements in the array
    for i in range(numElements):
        #gets the temp array
        tempVector = inputArray[:,i:(i+1)]
        #adds the temp vector to the list
        outputList.append(tempVector)

    #returns the temp vector
    return outputList


#defines the helper function to split up the control points into the first section of d, 
#middle section of M-d and last section of d
def getPartitionedControlPoints(controlPointsList: list[np.ndarray],
                                d: int,
                                M: int)->tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    
    #creates the initial list
    initial_list = []
    central_list = []
    end_list = []

    #iterates over all of the control points
    for i, controlPoint in enumerate(controlPointsList):
        
        #case we are in the range 0 to d
        if 0 <= i < d:
            initial_list.append(controlPoint)
        #case we are in the range d to M
        elif d <= i < M:
            central_list.append(controlPoint)
        #case we are in the range M to M+d
        elif M <= i < (M+d):
            end_list.append(controlPoint)

    
    #returns the three lists
    return initial_list, central_list, end_list

#creates a helper function that combines the control points together
def getControlPointsArray(controlPointsList: list[np.ndarray]):
    #gets the dimension of the control points
    tempPoint = controlPointsList[0]
    #gets the dimensions
    tempPointShape = np.shape(tempPoint)
    #gets the dimension
    numDimensions = tempPointShape[0]

    #iterates over the control points list members to get the array
    controlPointsArray = np.ndarray((numDimensions,0))

    #goes through and concatenates everything
    for controlPoint in controlPointsList:
        controlPointsArray = np.concatenate((controlPointsArray, controlPoint), axis=1)


    #returns the control point array
    return controlPointsArray