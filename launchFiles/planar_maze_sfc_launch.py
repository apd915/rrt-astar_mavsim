import os, sys
import numpy as np
from eVTOL_BSplines.message_types.msg_sfc import Msg_SFC
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarMazeParam, MapTypes
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.planar_maze_parameters as MAZE
import rrt_mavsim.parameters.aerosonde_parameters as AERO
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT
from rrt_mavsim.viewers.view_manager import ViewManager
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
from rrt_mavsim.viewers.plot_map_path import PlotMapPath
from rrt_mavsim.planners.bspline_generator import ObjectiveTypes
from bsplinegenerator.bsplines import BsplineEvaluation
import matplotlib.pyplot as plt
import time

viewer = ViewManager()

# gets the altitude
altitude = MAZE.altitude

# creates the plane message

startPosition_2D = MAZE.startPosition
endPosition_2D = MAZE.endPosition

path_gen = RRT_SFC_BSpline(
    numDimensions=MAZE.numDimensions,
    degree=FLIGHT.degree,
    M=FLIGHT.M,
    Va=PLAN.Va0,
    rho=FLIGHT.rho,
    step_length=FLIGHT.segmentLength,
    numDesiredInitPaths=FLIGHT.numInitialPaths,
    plane=MAZE.plane_msg,
    chiMax=FLIGHT.Chi_max,
)

#creates the planar Maze params
maze_params = PlanarMazeParam()

worldMap = MsgWorldMap(
    obstacleFieldType=MapTypes.PLANAR_MAZE,
    numDimensions_algorithm=MAZE.numDimensions,
    planarMazeParams=maze_params,
)

viewer.drawMap(world_map=worldMap)

pathGen_startTime = time.time()
path_gen.generateSFCPaths(
    startPosition_3D=MAZE.startPosition_3D,
    endPosition_3D=MAZE.endPosition_3D,
    worldMap=worldMap,
    segmentLength=FLIGHT.segmentLength,
)
pathGen_endTime = time.time()
pathGenTime = pathGen_endTime - pathGen_startTime
print("Path Gen Time: ", pathGenTime)

waypoints_not_smooth = path_gen.getWaypointsNotSmooth()
waypoints_smooth = path_gen.getWaypointsSmooth()

tree = path_gen.getTree()

controlPoints_minDistance_notSmooth = path_gen.generateControlPoints(
    waypoints=waypoints_not_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_DISTANCE
)
controlPoints_minVelocity_notSmooth = path_gen.generateControlPoints(
    waypoints=waypoints_not_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_VELOCITY
)
controlPoints_minAccel_notSmooth = path_gen.generateControlPoints(
    waypoints=waypoints_not_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_ACCELERATION
)

controlPoints_minDistance_smooth = path_gen.generateControlPoints(
    waypoints=waypoints_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_DISTANCE
)
controlPoints_minVelocity_smooth = path_gen.generateControlPoints(
    waypoints=waypoints_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_VELOCITY
)
controlPoints_minAccel_smooth = path_gen.generateControlPoints(
    waypoints=waypoints_smooth, numPointsPerUnit=FLIGHT.numPoints_perUnit, objectiveType=ObjectiveTypes.MIN_ACCELERATION
)

controlPointsList = [controlPoints_minDistance_smooth,
                     controlPoints_minVelocity_smooth,
                     controlPoints_minAccel_smooth]


#creates the B-Splines for the three smoothed control points lists
minDistance_bspline = BsplineEvaluation(control_points=controlPoints_minDistance_smooth,
                                        order=3,
                                        start_time=0.0)
minVelocity_bspline = BsplineEvaluation(control_points=controlPoints_minVelocity_smooth,
                                        order=3,
                                        start_time=0.0)
minAcceleration_bspline = BsplineEvaluation(control_points=controlPoints_minAccel_smooth,
                                        order=3,
                                        start_time=0.0)

##Section on generating the derivative data for each
minDistance_bspline_vel, timeData = minDistance_bspline.get_spline_derivative_data(100,1)
minDistance_bspline_accel, _ = minDistance_bspline.get_spline_derivative_data(100,2)
minDistance_bspline_curvature, _ = minDistance_bspline.get_spline_curvature_data(100)

minVelocity_bspline_vel, _ = minVelocity_bspline.get_spline_derivative_data(100,1)
minVelocity_bspline_accel, _ = minVelocity_bspline.get_spline_derivative_data(100,2)
minVelocity_bspline_curvature, _ = minVelocity_bspline.get_spline_curvature_data(100)

minAccel_bspline_vel, _ = minAcceleration_bspline.get_spline_derivative_data(100,1)
minAccel_bspline_accel, _ = minAcceleration_bspline.get_spline_derivative_data(100,2)
minAccel_bspline_curvature, _ = minAcceleration_bspline.get_spline_curvature_data(100)

minDistance_bspline_radius = 1/minDistance_bspline_curvature
minVelocity_bspline_radius = 1/minVelocity_bspline_curvature
minAccel_bspline_radius = 1/minAccel_bspline_curvature

fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True)

ax1.plot(timeData, np.linalg.norm(minDistance_bspline_vel, axis=0), label='Min Distance', color='green')
ax1.plot(timeData, np.linalg.norm(minVelocity_bspline_vel, axis=0), label='Min Velocity', color='gold')
ax1.plot(timeData, np.linalg.norm(minAccel_bspline_vel, axis=0), label='Min Accel', color='orange')
ax1.legend()
ax1.grid(True)
ax1.set_title("B-Spline Velocity Magnitudes")

ax2.plot(timeData, np.linalg.norm(minDistance_bspline_accel, axis=0), label='Min Distance', color='green')
ax2.plot(timeData, np.linalg.norm(minVelocity_bspline_accel, axis=0), label='Min Velocity', color='gold')
ax2.plot(timeData, np.linalg.norm(minAccel_bspline_accel, axis=0), label='Min Accel', color='orange')
ax2.legend()
ax2.grid(True)
ax2.set_title("B-Spline Acceleration Magnitudes")

ax3.plot(timeData, minDistance_bspline_curvature, label='Min Distance', color='green')
ax3.plot(timeData, minVelocity_bspline_curvature, label='Min Velocity', color='gold')
ax3.plot(timeData, minAccel_bspline_curvature, label='Min Accel', color='orange')
ax3.legend()
ax3.grid(True)
ax3.set_title("B-Spline Curvatures")

plt.show()


#plots the not smooth control points
viewer.update_planning_tree(
    waypoints_not_smooth=waypoints_not_smooth,
    waypoints_smooth=waypoints_smooth,
    tree=tree,
    degree=MAZE.degree,
    numdimensions=MAZE.numDimensions,
    R_ned_to_alt=MAZE.R,
    world_map=worldMap,
    controlPoints_list=controlPointsList,
    plane=MAZE.plane_msg,
)


plotter_noWaypoints = PlotMapPath(
    map=worldMap,
    controlPoints_not_smooth_list=None,
    plane=MAZE.plane_msg,
)

plotter_noWaypoints.plot(
    x_limits=MAZE.x_limits,
    y_limits=MAZE.y_limits,
    z_limits=MAZE.z_limits,
    aspectRatio=MAZE.aspect_ratio,
)

plotter_noSpline = PlotMapPath(
    map=worldMap,
    waypoints_not_smooth=waypoints_not_smooth,
    waypoints_smooth=waypoints_smooth,
    controlPoints_not_smooth_list=None,
    plane=MAZE.plane_msg,
)

plotter_noSpline.plot(
    x_limits=MAZE.x_limits,
    y_limits=MAZE.y_limits,
    z_limits=MAZE.z_limits,
    aspectRatio=MAZE.aspect_ratio,
)

plotter = PlotMapPath(
    map=worldMap,
    waypoints_smooth=waypoints_smooth,
    controlPoints_not_smooth_list=None,
    controlPoints_smooth_list=controlPointsList,
    plane=MAZE.plane_msg,
)

plotter.plot(
    x_limits=MAZE.x_limits,
    y_limits=MAZE.y_limits,
    z_limits=MAZE.z_limits,
    aspectRatio=MAZE.aspect_ratio,
)

testPoint = 0
