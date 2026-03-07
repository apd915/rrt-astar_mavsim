#this file contains all of the functions to obtain a smooth B-Spline from the RRT SFC action

import os, sys
import numpy as np
import cvxpy as cvp
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint, Bounds

from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.message_types.msg_flight_corridors import MsgFlightCorridor
from rrt_mavsim.tools.waypointsTools import getNumCntPts_list, getInitialFinalControlPoints, getNumCntPts
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PARAM
import rrt_mavsim.parameters.planner_parameters as PLAN
from rrt_mavsim.tools.convexConstraints import get_overlapping_control_points_constraints

from copy import deepcopy
from enum import Enum
 
class ObjectiveTypes(str, Enum):
    MIN_DISTANCE = "Minimize Distance"
    MIN_VELOCITY = "Minimize Velocity"
    MIN_ACCELERATION = "Minimize Acceleration"




class BSplineGenerator:


    def __init__(self,
                 numDimensions: int = 3,
                 num_intervals_free_space: int = 5,
                 degree: int = 3,
                 M: int = 10,
                 objective_type: ObjectiveTypes = ObjectiveTypes.MIN_DISTANCE,
                 Va: float = PLAN.Va0):

        self.numDimensions = numDimensions
        self.num_intervals_free_space = num_intervals_free_space
        self.degree = degree
        self.M = M
        self.objective_type = objective_type
        self.Va = Va

    def generatePath(self,
                     waypoints: MsgWaypoints_SFC,
                     numPointsPerUnit: float):
        
        #gets the list of number of control points
        self.numControlPoints_list = getNumCntPts_list(waypoints=waypoints,
                                                       numPointsPerUnit=numPointsPerUnit)
        
        #gets the total number of of control poins
        self.numControlPoints = getNumCntPts(numCntPts_list=self.numControlPoints_list,
                                             degree=self.degree)
        

        #gets the initial and final control points (2d total)
        startControlPoints, endControlPoints =\
            getInitialFinalControlPoints(corridorWaypoints=waypoints,
                                         Va=self.Va,
                                         degree=self.degree,
                                         M=self.M)
        
        #gets the cvxpy variable
        controlPoints_cpVar = cvp.Variable((self.numDimensions, self.numControlPoints))

        #gets the constraints for the cp var variable
        controlPoints_constraints = get_overlapping_control_points_constraints(controlPoints_var=controlPoints_cpVar,
                                                                               waypointData=waypoints,
                                                                               numCntPts_list=self.numControlPoints_list,
                                                                               startControlPoints=startControlPoints,
                                                                               endControlPoints=endControlPoints)

        #gets the objective functions
        if self.objective_type == ObjectiveTypes.MIN_DISTANCE:
            objectiveFunction = self.objective_minimum_distance(controlPoints_cpVar=controlPoints_cpVar)

        elif self.objective_type == ObjectiveTypes.MIN_VELOCITY:
            objectiveFunction = self.objective_minimum_velocity(controlPoints_cpVar=controlPoints_cpVar)

        elif self.objective_type == ObjectiveTypes.MIN_ACCELERATION:
            objectiveFunction = self.objective_minimum_acceleration(controlPoints_cpVar=controlPoints_cpVar)


        #section to solve the problem itself
        problem = cvp.Problem(objective=objectiveFunction,
                              constraints=controlPoints_constraints)

        #calls the solver
        problem.solve(solver=cvp.CLARABEL)

        outputControlPoints = controlPoints_cpVar.value

        return outputControlPoints


    #defines the minimum distance objective function
    def objective_minimum_distance(self,
                                   controlPoints_cpVar: cvp.Variable):
        
        #gets the velocity control poitns
        velocityControlPoints_cp = controlPoints_cpVar[:,0:-1] - controlPoints_cpVar[:,1:]

        minimizeLength_objectiveFunction = cvp.Minimize(cvp.sum(cvp.norm(velocityControlPoints_cp, axis=1)))

        #returns the objective
        return minimizeLength_objectiveFunction

    #define sthe objective function to minimize velocity
    def objective_minimum_velocity(self,
                                   controlPoints_cpVar: cvp.Variable):
        #gets the acceleration control points (which we will use to minimize velocitu)
        accelerationControlPoints_cp =  controlPoints_cpVar[:,2:] - 2*controlPoints_cpVar[:,1:-1] + controlPoints_cpVar[:,0:-2]

        minimizeVelocity_objectiveFunction = cvp.Minimize(cvp.sum(cvp.norm(accelerationControlPoints_cp, axis=1)))


        return minimizeVelocity_objectiveFunction

    #defines the objective function to minimize Acceleration
    def objective_minimum_acceleration(self,
                                       controlPoints_cpVar: cvp.Variable):

        jerkControlPoints_cp = controlPoints_cpVar[:,3:] - 3*controlPoints_cpVar[:,2:-1] + 3*controlPoints_cpVar[:,1:-2] - controlPoints_cpVar[:,0:-3]

        minimizeAcceleration_objectiveFunction = cvp.Minimize(cvp.sum(cvp.norm(jerkControlPoints_cp, axis=1)))

        return minimizeAcceleration_objectiveFunction
        



    

