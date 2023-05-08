import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan, arctan2
import numpy as np
from motion_planner import compute_initial_point, compute_trajectory, planner, planner_corridor_sequence
# from corridor import Corridor
from corridor_world import CorridorWorld


#PARAMETERS
# v_max = 2
# v_min = -2
# omega_max = 2
# omega_min = -2
v_max = 0.5
v_min = -0.5
omega_max = 1.57
omega_min = -1.57
u_bounds = np.array([v_min, v_max, omega_min, omega_max])
a = 0.430
b = 0.508
m = 0.1

#######################################################################
# Run this test by using `python3 test_compute_traj.py {test_number}` 
# where {test_number} can be replaced by an integer.
#######################################################################

import sys
test = int(sys.argv[1]) if len(sys.argv) > 1 else 1

# The local tilt angle (wrt the body frame) is measured wrt the forward direction (x-axis in the body frame)
# To use the compute_trajectory function (by Sonia) the tilt angle must, however, be measured from the right direction (x-axis in the world frame)

#################################################
# Test 1: Turn left and then right (3 corridors)
#################################################
if test == 1:
    print("Testing Arena case (3 corridors)")
    # vehicle pose
    veh_posx = 1.5
    veh_posy = 2.8 
    veh_tilt = 0 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1.05
    height1 = 11.7
    width2 = 1.4
    height2 = 13.8
    width3 = 1.1
    height3 = 17.4

    # In local frame
    tilt1_local = 0.33         # With respect to the vehicle
    tilt2_local = tilt1_local + 1.02 # With respect to the vehicle
    tilt3_local = tilt2_local - 1.1 # With respect to the vehicle


# In world frame
tilt1_world = veh_tilt + tilt1_local
tilt2_world = veh_tilt + tilt2_local
tilt3_world = veh_tilt + tilt3_local

# # Compute center of the corridors wrt world frame
# center1_world = np.array([veh_posx, veh_posy]) + np.array([-(height1/2)*sin(tilt1_world), (height1/2)*cos(tilt1_world)])
# center2_world = center1_world + np.array([-0.5*sin(tilt1_world)-(height2/2)*sin(tilt2_world), 0.5*cos(tilt1_world)+(height2/2)*cos(tilt2_world)])
# center3_world = center2_world + np.array([-0.5*sin(tilt2_world)-(height3/2)*sin(tilt3_world), 0.5*cos(tilt2_world)+(height3/2)*cos(tilt3_world)])
# Compute center of the corridors wrt world frame
center1_world = np.array([veh_posx, veh_posy]) + np.array([-(height1/2)*sin(tilt1_world), (height1/2)*cos(tilt1_world)])
center2_world = center1_world + np.array([-0.5*sin(tilt1_world)-(0.44*height2)*sin(tilt2_world), 0.40*cos(tilt1_world)+(0.13*height2)*cos(tilt2_world)])
center3_world = center2_world + np.array([-0.9*sin(tilt2_world)-(1.01*height3)*sin(tilt3_world), 0.5*cos(tilt2_world)+(0.5*height3)*cos(tilt3_world)])

corridor1_world = CorridorWorld(width1, height1, center1_world, tilt1_world)
corridor2_world = CorridorWorld(width2, height2, center2_world, tilt2_world)
corridor3_world = CorridorWorld(width3, height3, center3_world, tilt3_world)

initial_point = compute_initial_point(corridor1_world, m)
x0 = initial_point[0]
y0 = initial_point[1]

init_pos = [1.45, 3.03]
goal_pos = [-11.64, 18.37]

# corridor2_world = None
#sequence_man, computed_path = planner(corridor1_world, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = False, corridor2 = corridor2_world)
# sequence_man = planner(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True)
corridor_list = [corridor1_world, corridor2_world, corridor3_world]
maneuver_list, computed_path_list, poses_sequence_list= planner_corridor_sequence(corridor_list, u_bounds, a, b, m, False, x0, y0, veh_tilt+pi/2, xf = goal_pos[0], yf = goal_pos[1])
print(maneuver_list)
print(poses_sequence_list)




def plot_result_in_world(path, *args):
    '''This function will plot any corridors provided as arguments'''
    print("Plotting with image")
    figure_solution = plt.figure()

    ax = figure_solution.add_subplot(111)

    # Plot initial and final point
    plt.scatter([init_pos[0], goal_pos[0]], [init_pos[1], goal_pos[1]], s=100, c=['g','r'], zorder=10)

    # Plot world image
    im = plt.imread("world_grid.png")
    # im = plt.imread("world_grid_path.png")
    implot = plt.imshow(im, extent=(-24,36,-4,28), alpha=1, zorder=1)
    
    # Plot global path
    global_path = np.array([init_pos,[-0.5464,7.1233],[-9.126,10.39],[-9.404,10.59],[-9.542,10.876],goal_pos])
    plt.plot(global_path[:,0],global_path[:,1], 'm', linewidth=1.5, zorder=2)

    # Plot corridors
    for corridor in args:
        corners = corridor.corners
        corners.append(corners[0])
        plt.plot([corner[0] for corner in corners], 
                [corner[1] for corner in corners],
                'k--', linewidth=1)

    # # Plot path
    if path is not None:
        plt.plot(path[:,0],path[:,1], 'g', linewidth=1.5, zorder=2)



    ax.set_aspect('equal')
    plt.show(block=True)

    
# plot_result_in_world(computed_path_list, corridor1_world)
# plot_result_in_world(computed_path_list, corridor1_world, corridor2_world)
# plot_result_in_world(computed_path_list, corridor1_world, corridor2_world, corridor3_world)


# import imageio
# gif_name = 'arena_ugm2.gif'
# filenames = [
#     'Figure_1.png',
#     'Figure_2.png',
#     'Figure_3.png',
#     'Figure_4.png',
#     'Figure_5a.png',
#     'Figure_5b.png',
#     'Figure_5c.png',
#     'Figure_6.png',
#     'Figure_7.png'
#     ]
# images = []
# for filename in filenames:
#     images.append(imageio.imread(filename))
# # imageio.mimsave(gif_name, images)

# # Save them as frames into a gif 
# kargs = { 'duration': 2000, 'loop': 0 }
# imageio.mimsave(gif_name, images, 'GIF', **kargs)

