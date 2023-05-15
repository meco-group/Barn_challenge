import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan, arctan2
import numpy as np
from motion_planner import compute_initial_point, compute_trajectory, planner, planner_corridor_sequence
# from corridor import Corridor
from corridor_world import CorridorWorld


#PARAMETERS
v_max = 4
v_min = -4
omega_max = 2
omega_min = -2
# v_max = 0.5
# v_min = -0.5
# omega_max = 1.57
# omega_min = -1.57
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
maneuver_list, computed_path_list, poses_sequence_list= planner_corridor_sequence(corridor_list, u_bounds, a, b, m, False, init_pos[0], init_pos[1], veh_tilt+pi/2, xf = goal_pos[0], yf = goal_pos[1])
print(maneuver_list)
print(poses_sequence_list)
#FIRST PART ENDS HERE------------------------------------


#SECOND PART---------------------------------------------
def plot_primitives(maneuver_list, **kwargs):
    primitives_number = kwargs['primitives_number'] if ('primitives_number' in kwargs and kwargs['primitives_number'] is not None) else len(maneuver_list)
    figure_forward_velocity = plt.figure()
    ax = figure_forward_velocity.add_subplot(211)
    plt.xlim([0, 8])
    #plt.title('Forward velocity')
    #plt.xlabel('Time [s]')
    plt.ylabel('V [m/s]')
    init_time = 0
    color = '#030764'
    for i in range(primitives_number):
        plt.plot([init_time, init_time + maneuver_list[i,2]], [maneuver_list[i,0], maneuver_list[i,0]], color)
        init_time = init_time + maneuver_list[i,2]
        if color == '#030764':
            color = '#F97306'
        else:
            color = '#030764'
    #figure_angular_velocity = plt.figure()
    ax = figure_forward_velocity.add_subplot(212)
    plt.xlim([0, 8])
    plt.ylim([-2.2, 2.2])

    #plt.title('Angular velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Omega [rad/s]')
    init_time = 0
    color = '#030764'
    init_vel = maneuver_list[i,1]
    for i in range(primitives_number):
        plt.plot([init_time, init_time, init_time + maneuver_list[i,2]], [init_vel, maneuver_list[i,1], maneuver_list[i,1]], color)
        init_time = init_time + maneuver_list[i,2]
        init_vel = maneuver_list[i,1]
        if color == '#030764':
            color = '#F97306'
        else:
            color = '#030764'
    plt.show(block=True)
    return figure_forward_velocity

    
def plot_result_in_world(path,  primitives_number, *args):
    '''This function will plot any corridors provided as arguments'''
    print("Plotting with image")
    figure_solution = plt.figure()

    ax = figure_solution.add_subplot(111)

    # Plot initial and final point
    plt.scatter([init_pos[0], goal_pos[0]], [init_pos[1], goal_pos[1]], s=10, c=['g','r'], zorder=10)

    # Plot world image
    #im = plt.imread("world_grid.png")
    # im = plt.imread("world_grid_path.png")
    #implot = plt.imshow(im, extent=(-24,36,-4,28), alpha=1, zorder=1)
    
    # Plot global path
    global_path = np.array([init_pos,[-0.5464,7.1233],[-9.126,10.39],[-9.404,10.59],[-9.542,10.876],goal_pos])
    #plt.plot(global_path[:,0],global_path[:,1], 'm', linewidth=1.5, zorder=2)

    # Plot corridors
    for corridor in args:
        corners = corridor.corners
        corners.append(corners[0])
        plt.plot([corner[0] for corner in corners], 
                [corner[1] for corner in corners],
                'k--', linewidth=1)
    #plt.legend()
    # Plot path
    if path is not None:
        #plt.plot(path[:,0],path[:,1], 'g', linewidth=1.5, zorder=2)
        if primitives_number == 1:
            plt.plot(path[0:100,0],path[0:100,1], '#030764', linewidth=1.5, zorder=2)
        elif primitives_number ==2: 
            plt.plot(path[0:100,0],path[0:100,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[100:102,0],path[100:102,1], '#F97306', linewidth=1.5, zorder=2)
        elif primitives_number == 3:
            plt.plot(path[0:100,0],path[0:100,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[100:102,0],path[100:102,1], '#F97306', linewidth=1.5, zorder=2)
            plt.plot(path[102:203,0],path[102:203,1], '#030764', linewidth=1.5, zorder=2)
        elif primitives_number == 4:
            plt.plot(path[0:100,0],path[0:100,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[100:102,0],path[100:102,1], '#F97306', linewidth=1.5, zorder=2)
            plt.plot(path[102:203,0],path[102:203,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[203:205,0],path[203:205,1], '#F97306', linewidth=1.5, zorder=2)
        elif primitives_number == 5: 
            plt.plot(path[0:100,0],path[0:100,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[100:102,0],path[100:102,1], '#F97306', linewidth=1.5, zorder=2)
            plt.plot(path[102:203,0],path[102:203,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[203:205,0],path[203:205,1], '#F97306', linewidth=1.5, zorder=2)
            plt.plot(path[205:305,0],path[205:305,1], '#030764', linewidth=1.5, zorder=2)
        else:
            plt.plot(path[0:100,0],path[0:100,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[100:102,0],path[100:102,1], '#F97306', linewidth=1.5, zorder=2)
            plt.plot(path[102:203,0],path[102:203,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[203:205,0],path[203:205,1], '#F97306', linewidth=1.5, zorder=2)
            plt.plot(path[205:305,0],path[205:305,1], '#030764', linewidth=1.5, zorder=2)
            plt.plot(path[305:307,0],path[305:307,1], '#F97306', linewidth=1.5, zorder=2)

    ax.set_aspect('equal')
    plt.show(block=True)
    return figure_solution

    
# plot_result_in_world(computed_path_list, corridor1_world)
# plot_result_in_world(computed_path_list, corridor1_world, corridor2_world)
plot_result_in_world(computed_path_list, 6, corridor1_world, corridor2_world, corridor3_world)

corridor_margin1 = CorridorWorld(corridor1_world.width-(a+2*m), corridor1_world.height-2*m, corridor1_world.center, corridor1_world.tilt)
corridor_margin2 = CorridorWorld(corridor2_world.width-(a+2*m), corridor2_world.height-2*m, corridor2_world.center, corridor2_world.tilt)
corridor_margin3 = CorridorWorld(corridor3_world.width-(a+2*m), corridor3_world.height-2*m, corridor3_world.center, corridor3_world.tilt)
                
# plot_result_in_world(computed_path_list, corridor1_world, corridor2_world, corridor3_world, corridor_margin1, corridor_margin2, corridor_margin3)
#######################
generate_gif = True
file_name_path = ['path1.png', 'path2.png', 'path3.png', 'path4.png', 'path5.png', 'path6.png']
file_name_controls = ['controls1.png', 'controls2.png','controls3.png','controls4.png','controls5.png','controls6.png' ]
if generate_gif:
    for primitives_number in range(len(maneuver_list)):
        path_figure = plot_result_in_world(computed_path_list, primitives_number + 1, corridor1_world, corridor2_world, corridor3_world)
        path_figure.savefig(file_name_path[primitives_number])
        primitives_figure =plot_primitives(maneuver_list, primitives_number = primitives_number + 1)
        primitives_figure.savefig(file_name_controls[primitives_number])


plot_primitives(maneuver_list, primitives_number = 6)

import imageio
gif_name = 'primitives.gif'
filenames = [
    'path1.png',
    'path2.png',
    'path3.png',
    'path4.png',
    'path5.png',
    'path6.png',
    ]
images = []
for filename in filenames:
    images.append(imageio.imread(filename))
# # imageio.mimsave(gif_name, images)

# # Save them as frames into a gif 
kargs = { 'duration': 1000, 'loop': 0 }
imageio.mimsave(gif_name, images, 'GIF', **kargs)

gif_name = 'controls.gif'
filenames = [
    'controls1.png',
    'controls2.png',
    'controls3.png',
    'controls4.png',
    'controls5.png',
    'controls6.png',
    ]
images = []
for filename in filenames:
    images.append(imageio.imread(filename))
# # imageio.mimsave(gif_name, images)

# # Save them as frames into a gif 
kargs = { 'duration': 1000, 'loop': 0 }
imageio.mimsave(gif_name, images, 'GIF', **kargs)

