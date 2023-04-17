import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan, arctan2
from motion_planner import compute_initial_point, compute_trajectory
from corridor import *

from barn_challenge.msg import corridor_msg
from nav_msgs.msg import Odometry
from corridor_manager import transformCorridorToWorld, odomMsgClass

def convertCorridorToCorridorMsg(corr):
    new_message = corridor_msg()
    new_message.height = corr.height
    new_message.width = corr.width
    new_message.quality = corr.quality
    new_message.center = corr.center
    new_message.growth_center = corr.growth_center
    new_message.tilt = corr.tilt
    new_message.corners = corr.corners
    return new_message

def convertOdometryToOdometryMsg(posx, posy, theta):
    curr_pose = odomMsgClass()
    curr_pose.posx = posx
    curr_pose.posy = posy
    curr_pose.theta = theta
    return curr_pose


#PARAMETERS
v_max = 2
v_min = -2
omega_max = 2
omega_min = -2
u_bounds = np.array([v_min, v_max, omega_min, omega_max])
a = 0.430
b = 0.508
m = 0.3

#######################################################################
# Run this test by using `python3 test_compute_traj.py {test_number}` 
# where {test_number} can be replaced by an integer.
#######################################################################

import sys
test = int(sys.argv[1]) if len(sys.argv) > 1 else 1

# The local tilt angle (wrt the body frame) is measured wrt the forward direction (x-axis in the body frame)
# To use the compute_trajectory function (by Sonia) the tilt angle must, however, be measured from the right direction (x-axis in the world frame)

#####################################
# Test 1: Turn left and then left
#####################################
if test == 1:
    print("Testing left-left")
    # vehicle pose
    veh_posx = 1
    veh_posy = 3 
    veh_tilt = -pi/4 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 5.5
    width2 = 1
    height2 = 5.5
    # In local frame
    tilt1_local = pi/4         # With respect to the vehicle
    tilt2_local = pi/4 + pi/5 # With respect to the vehicle
#####################################
# Test 2: Turn right and then left
#####################################
elif test == 2: 
    print("Testing right-left")
    # vehicle pose
    veh_posx = 1
    veh_posy = 3 
    veh_tilt = pi/4 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 5.5
    width2 = 1
    height2 = 5.5
    # In local frame
    tilt1_local = -pi/4         # With respect to the vehicle
    tilt2_local = -pi/4 + pi/5 # With respect to the vehicle
#####################################
# Test 3: Turn left and then right
#####################################
elif test == 3:
    print("Testing left-right")
    # vehicle pose
    veh_posx = 1
    veh_posy = 3 
    veh_tilt = -pi/4 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 5.5
    width2 = 1
    height2 = 5.5
    # In local frame
    tilt1_local = pi/4        # With respect to the vehicle
    tilt2_local = pi/4 - pi/5 # With respect to the vehicle
#####################################
# Test 4: Turn right and then right
#####################################
elif test == 4:
    print("Testing right-right")
    # vehicle pose
    veh_posx = 1
    veh_posy = 3 
    veh_tilt = pi/4 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 5.5
    width2 = 1
    height2 = 5.5
    # In local frame
    tilt1_local = -pi/4        # With respect to the vehicle
    tilt2_local = -pi/4 - pi/5 # With respect to the vehicle


# Compute center of the corridors wrt body frame
center1_local = np.array([-(height1/2)*sin(tilt1_local), (height1/2)*cos(tilt1_local)])
center2_local = center1_local + np.array([-2.0*sin(tilt1_local)-(height2/2)*sin(tilt2_local), 2.0*cos(tilt1_local)+(height2/2)*cos(tilt2_local)])

corridor1_local = Corridor( width1, height1, center1_local, tilt1_local+pi/2) # Notice the addition of pi/2
corridor2_local = Corridor( width2, height2, center2_local, tilt2_local+pi/2)

# In world frame
tilt1_world = veh_tilt + tilt1_local
tilt2_world = veh_tilt + tilt2_local

# Compute center of the corridors wrt world frame
center1_world = np.array([veh_posx, veh_posy]) + np.array([-(height1/2)*sin(tilt1_world), (height1/2)*cos(tilt1_world)])
center2_world = center1_world + np.array([-2.0*sin(tilt1_world)-(height2/2)*sin(tilt2_world), 2.0*cos(tilt1_world)+(height2/2)*cos(tilt2_world)])

corridor1_world = Corridor(width1, height1, center1_world, tilt1_world+pi/2) # Notice the addition of pi/2
corridor2_world = Corridor(width2, height2, center2_world, tilt2_world+pi/2)

# Check conversion from local to world frame (same as implemented in corridor_manager)
robot_odometry = convertOdometryToOdometryMsg(veh_posx, veh_posy, veh_tilt)
corridor1_converted = transformCorridorToWorld(convertCorridorToCorridorMsg(corridor1_local.corridor_copy()), robot_odometry)
corridor2_converted = transformCorridorToWorld(convertCorridorToCorridorMsg(corridor2_local.corridor_copy()), robot_odometry)


initial_point = compute_initial_point(corridor1_world, m)
x0 = initial_point[0]
y0 = initial_point[1]


# Use compute_trajectory (by Sonia). Be aware that the tilt angle of the vehicle should be measured from the x-axis of the world frame
sequence_man = compute_trajectory(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True, corridor2 = corridor2_converted)
# sequence_man = compute_trajectory(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True)

def plot_corridors(*args):
    '''This function will plot any corridors provided as arguments'''
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
