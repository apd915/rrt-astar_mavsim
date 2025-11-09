#this file implements convexity and constraints for the safe flight corridors
import cvxpy as cvp
import numpy as np
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.message_types.msg_safeFlightCorridor import Msg_SFC
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PARAM


#defines the function to create overlapping control point constraints
def get_overlapping_control_points_constraints(controlPoints_var: cvp.Variable,
                                               waypointData: MsgWaypoints_SFC,
                                               numCntPts_list: list[int],
                                               startControlPoints: np.ndarray,
                                               endControlPoints: np.ndarray,
                                               degree: int = FLIGHT_PARAM.degree):
    
    #obstains the number of dimensions 
    num_dimensions = waypointData.numDimensions
    #gets the flight corridor list
    flightCorridorList = waypointData.getAllFlightCorridors()
    #gets the corresponding SFC list
    sfc_list = waypointData.getAllSFCs()

    #sets the complete list of A and b matrices. Each element contains another list of all of the A and b matrices
    #that applies to that particular control point
    A_matrices_complete = []
    b_vectors_complete = []

    #sets the index for accessing the control points indices
    controlPoints_index = 0

    #variable list for the control points constraints
    controlPoints_constraints = []

    #iterates over all of the sfcs
    for i, (numCntPts, sfc) in enumerate(zip(numCntPts_list, sfc_list)):

        #gets the incremental index
        incremental_index = numCntPts - degree

        #gets the current control points partition
        controlPointsPartition \
              = controlPoints_var[:,(controlPoints_index):(controlPoints_index+numCntPts)]

        #gets the A and b matrices for the control points partition
        A, b = sfc.getAbMatrices()



        #iterates over the control points in the current corridor
        for i in range(numCntPts):
            #current index
            currentIndex = controlPoints_index + i

            #checks if the index currently exists in the A list
            if currentIndex < len(A_matrices_complete):
                #then we can add to it
                #gets the current A and b lists
                A_matrices_complete[currentIndex].append(A)
                b_vectors_complete[currentIndex].append(b)
            #otherwise we create it
            else:
                A_matrices_complete.append([A])
                b_vectors_complete.append([b])

            potato = 0
        
        #gets the inequality constraints
        inequalityConstraint_temp = [A @ controlPointsPartition <= b]

        controlPoints_constraints += inequalityConstraint_temp

        controlPoints_index +=incremental_index


    #gets the A concatenated list
    A_list_cat = concatenateArrayList(matrixList=A_matrices_complete)
    b_list_cat = concatenateArrayList(matrixList=b_vectors_complete)


    ###############################################################
    #equality constraints section
    
    #the section of the cp var that denotes the control points
    startControlPoints_cpVar = controlPoints_var[:, :degree]
    endControlPoints_cpVar = controlPoints_var[:, (-degree):]

    #creates the equality constraints for start and end conditions
    startEqualityConstraint = [startControlPoints_cpVar == startControlPoints]
    endEqualityConstraint = [endControlPoints_cpVar == endControlPoints]

    #adds these equality constraints to the main constraints list
    controlPoints_constraints += startEqualityConstraint
    controlPoints_constraints += endEqualityConstraint


    #return the control points constraints
    return controlPoints_constraints





def concatenateArrayList(matrixList: list[list[np.ndarray]]):

    #gets the shape of the first matrix
    primaryMatrix = (matrixList[0])[0]

    primaryMatrixShape = np.shape(primaryMatrix)

    #gets the number of dimensions of the state vector
    numDimensions = primaryMatrixShape[1]

    concatenatedMatrixList = []

    for sublist in matrixList:

        A_temp = np.ndarray((0, numDimensions))

        for matrix_temp in sublist:
            
            #concatenates to the A_temp
            A_temp = np.concatenate((A_temp, matrix_temp), axis=0)

        #appends onto the matrix list
        concatenatedMatrixList.append(A_temp)

    #returns the concatenated matrix list
    return concatenatedMatrixList