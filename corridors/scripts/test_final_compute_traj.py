import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan, arctan2
from motion_planner import get_corner_point, compute_goal_point, compute_initial_point, compute_trajectory
from corridor import *

def transformCorridorToWorld2(newCorridor, posx, posy, theta):

    L = sqrt(newCorridor.center[0]**2 + newCorridor.center[1]**2)
    phi = arctan2(newCorridor.center[1],newCorridor.center[0]) - pi/2  + theta 
    newCorridor.center = [posx - L*sin(phi), posy + L*cos(phi)]

    L_growth = sqrt(newCorridor.growth_center[0]**2 + newCorridor.growth_center[1]**2)
    phi_growth = arctan2(newCorridor.growth_center[1],newCorridor.growth_center[0]) - pi/2  + theta 
    newCorridor.growth_center = [posx - L_growth*sin(phi_growth), posy + L_growth*cos(phi_growth)]

    newCorridor.tilt = newCorridor.tilt + theta

    newCorridor.update_W()

    newCorridor.corners = newCorridor.get_corners()
    
    return newCorridor


#PARAMETERS
v_max = 2
v_min = -2
omega_max = 2
omega_min = -2
u_bounds = np.array([v_min, v_max, omega_min, omega_max])
a = 0.430
b = 0.508
m = 0.3
#test1: turn left, orientation = pi/3 < 90 
#test2: turn left, orientation = 2*pi/3 > 90
#test3: turn right, orientation = pi/3 <90
#test4: turn right, orientatiom = 2*pi/3 >90
#test5: turn left, orientation = pi/3 (same test1), L-shaped turn
#test6: turn right, orientation =  2*pi/3 (same test4), L-shaped turn

# test = 1
# #DEFINE TWO CORRIDORS
# if test ==1:
#     theta0 = pi/3
#     tilt1 = 0
#     tilt2 = pi/12
#     corridor1 = Corridor( 1, 5.5,np.array([0.5, 5.5/2]),tilt1)
#     corridor2 = Corridor( 1, 5.5,np.array([0.5 - cos(abs(tilt2)), 4.5 + 5.5 *sin(abs(tilt2))]),tilt2)
# elif test == 2:
#     theta0 = 2*pi/3
#     tilt1 = -pi/12
#     tilt2 = pi/6
#     corridor1 = Corridor( 1, 5.5,np.array([0.5, 5.5/2]),tilt1)
#     corridor2 = Corridor( 1, 5.5,np.array([0.5 - cos(abs(tilt2)), 4.5 + 5.5 *sin(abs(tilt2))]),tilt2)
# elif test == 3:
#     theta0 = pi/3
#     tilt1 = 0
#     tilt2 = -pi/12
#     corridor1 = Corridor( 1, 5.5,np.array([0.5, 5.5/2]),tilt1)
#     corridor2 = Corridor( 1.5, 5.5,np.array([1.5 - cos(abs(tilt2)), 5 + 5.5 *sin(abs(tilt2))]),tilt2)
# elif test == 4:
#     theta0 = 2*pi/3
#     tilt1 = 0
#     tilt2 = -pi/12
#     corridor1 = Corridor( 1, 5.5,np.array([0.5, 5.5/2]),tilt1)
#     corridor2 = Corridor( 1.5, 5.5,np.array([1.5 - cos(abs(tilt2)), 5 + 5.5 *sin(abs(tilt2))]),tilt2)
# elif test ==5:
#     theta0 = pi/3
#     tilt1 = 0
#     tilt2 = pi/2
#     corridor1 = Corridor( 1, 5.5,np.array([0.5, 5.5/2]),tilt1)
#     corridor2 = Corridor( 1, 5.5,np.array([-2.5, 5]),tilt2)
# elif test ==6:
#     theta0 = 2*pi/3
#     tilt1 = 0
#     tilt2 = -pi/2
#     corridor1 = Corridor( 1, 5.5,np.array([0.5, 5.5/2]),tilt1)
#     corridor2 = Corridor( 1, 5.5,np.array([2.5, 5]),tilt2)

test = 1
#DEFINE TWO CORRIDORS
if test ==1:
    # vehicle pose
    veh_posx = 1
    veh_posy = 3 
    veh_tilt = -pi/8 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 5.5
    width2 = 1
    height2 = 5.5
    # In local frame
    tilt1_local = pi/8         # With respect to the vehicle
    tilt2_local = pi/8 + pi/12 # With respect to the vehicle

center1_local = np.array([-(height1/2)*sin(abs(tilt1_local)), (height1/2)*cos(abs(tilt1_local))])
center2_local = center1_local + np.array([-2.0*sin(tilt1_local)-(height2/2)*sin(abs(tilt2_local)), 2.0*cos(tilt1_local)+(height2/2)*cos(abs(tilt2_local))])

corridor1_local = Corridor( width1, height1, center1_local, tilt1_local+pi/2)
corridor2_local = Corridor( width2, height2, center2_local, tilt2_local+pi/2)

# In world frame
tilt1_world = veh_tilt + tilt1_local
tilt2_world = veh_tilt + tilt2_local

center1_world = np.array([veh_posx, veh_posy]) + np.array([-(height1/2)*sin(tilt1_world), (height1/2)*cos(tilt1_world)])
center2_world = center1_world + np.array([-2.0*sin(tilt1_world)-(height2/2)*sin(tilt2_world), 2.0*cos(tilt1_world)+(height2/2)*cos(tilt2_world)])

corridor1_world = Corridor(width1, height1, center1_world, tilt1_world+pi/2)
corridor2_world = Corridor(width2, height2, center2_world, tilt2_world+pi/2)

corridor1_converted = transformCorridorToWorld2(corridor1_local.corridor_copy(), veh_posx, veh_posy, veh_tilt)
corridor2_converted = transformCorridorToWorld2(corridor2_local.corridor_copy(), veh_posx, veh_posy, veh_tilt)


initial_point = compute_initial_point(corridor1_world, m)
x0 = initial_point[0]
y0 = initial_point[1]

sequence_man = compute_trajectory(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True, corridor2 = corridor2_converted)

def plot_corridors(*args):
    #Plot solution
    figure_solution = plt.figure()
    ax = figure_solution.add_subplot(111)

    for corridor in args:
        corners = corridor.corners
        corners.append(corners[0])
        plt.plot([corner[0] for corner in corners], 
                [corner[1] for corner in corners],
                'k--', linewidth=1)

    ax.set_aspect('equal')
    plt.show(block=True)

# plot_corridors(corridor1_local, corridor2_local) 
# plot_corridors(corridor1_world, corridor2_world)
# plot_corridors(corridor1_converted, corridor2_converted, corridor1_world, corridor2_world)
# a=1
