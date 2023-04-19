import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan, arctan2
import numpy as np
from motion_planner import compute_initial_point, compute_trajectory, planner
from corridor import Corridor
from corridor_world import CorridorWorld



USE_ROS = False


if USE_ROS:
    from barn_challenge.msg import CorridorLocalMsg
    from nav_msgs.msg import Odometry
    from corridor_manager import transform_corridor_to_world, odomMsgClass

    def convertCorridorToCorridorMsg(corr, posx, posy, theta):
        new_message = CorridorLocalMsg()
        new_message.height_local = corr.height
        new_message.width_local = corr.width
        new_message.quality_local = corr.quality
        new_message.center_local = corr.center
        new_message.growth_center_local = corr.growth_center
        new_message.tilt_local = corr.tilt
        new_message.corners_local = corr.corners
        new_message.init_pos_global = [posx, posy, theta]
        return new_message

    # def convertOdometryToOdometryMsg(posx, posy, theta):
    #     curr_pose = odomMsgClass()
    #     curr_pose.posx = posx
    #     curr_pose.posy = posy
    #     curr_pose.theta = theta
    #     return curr_pose


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
    veh_posx = 0
    veh_posy = 0 
    veh_tilt = -pi/4 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 2.5
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
#####################################
# Test 5: Turn right and then right (shorter)
#####################################
elif test == 5:
    print("Testing right-right (shorter)")
    # vehicle pose
    veh_posx = 1
    veh_posy = 3
    veh_tilt = pi/8 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 2
    width2 = 1
    height2 = 1.5
    # In local frame
    tilt1_local = -pi/8        # With respect to the vehicle
    tilt2_local = -pi/8 - pi/5 # With respect to the vehicle
#####################################
# Test 6: Turn right and then right
#####################################
elif test == 6:
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
#####################################
# Test 7: Turn right and then right
#####################################
elif test == 7:
    print("Testing right-right")
    # vehicle pose
    veh_posx = 1
    veh_posy = 3 
    veh_tilt = -pi/2 # Vehicle tilt with respect to the world frame
    # corridors
    width1 = 1
    height1 = 5.5
    width2 = 1
    height2 = 5.5
    # In local frame
    tilt1_local = pi/2        # With respect to the vehicle
    tilt2_local = pi/2 - pi/5 # With respect to the vehicle

# Compute center of the corridors wrt body frame
center1_local = np.array([-(height1/2)*sin(tilt1_local), (height1/2)*cos(tilt1_local)])
center2_local = center1_local + np.array([-0.5*sin(tilt1_local)-(height2/2)*sin(tilt2_local), 0.5*cos(tilt1_local)+(height2/2)*cos(tilt2_local)])

corridor1_local = Corridor( width1, height1, center1_local, tilt1_local+pi/2) # Notice the addition of pi/2
corridor2_local = Corridor( width2, height2, center2_local, tilt2_local+pi/2)

# In world frame
tilt1_world = veh_tilt + tilt1_local
tilt2_world = veh_tilt + tilt2_local

# Compute center of the corridors wrt world frame
center1_world = np.array([veh_posx, veh_posy]) + np.array([-(height1/2)*sin(tilt1_world), (height1/2)*cos(tilt1_world)])
center2_world = center1_world + np.array([-0.5*sin(tilt1_world)-(height2/2)*sin(tilt2_world), 0.5*cos(tilt1_world)+(height2/2)*cos(tilt2_world)])

corridor1_world = CorridorWorld(width1, height1, center1_world, tilt1_world) # Notice the addition of pi/2
corridor2_world = CorridorWorld(width2, height2, center2_world, tilt2_world)

initial_point = compute_initial_point(corridor1_world, m)
x0 = initial_point[0]
y0 = initial_point[1]
# x0 = 1.43
# y0 = 7.2

if USE_ROS:
    # Check conversion from local to world frame (same as implemented in corridor_manager)
    # robot_odometry = convertOdometryToOdometryMsg(veh_posx, veh_posy, veh_tilt)
    corridor1_converted = transform_corridor_to_world(convertCorridorToCorridorMsg(corridor1_local.corridor_copy(), veh_posx, veh_posy, veh_tilt))
    corridor2_converted = transform_corridor_to_world(convertCorridorToCorridorMsg(corridor2_local.corridor_copy(), veh_posx, veh_posy, veh_tilt))

    # corridor2_converted = None
    # Use compute_trajectory (by Sonia). Be aware that the tilt angle of the vehicle should be measured from the x-axis of the world frame
    sequence_man, computed_path = planner(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True, corridor2 = corridor2_converted)
    # sequence_man = planner(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True)
else:
    # corridor2_world = None
    sequence_man, computed_path = planner(corridor1_world, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True, corridor2 = corridor2_world)
    # sequence_man = planner(corridor1_converted, u_bounds, a, b, m, x0, y0, veh_tilt+pi/2, plot = True)

print(sequence_man)

def plot_corridors(path, *args):
    '''This function will plot any corridors provided as arguments'''
    figure_solution = plt.figure()
    ax = figure_solution.add_subplot(111)

    for corridor in args:
        corners = corridor.corners
        corners.append(corners[0])
        plt.plot([corner[0] for corner in corners], 
                [corner[1] for corner in corners],
                'k--', linewidth=1)

    if path is not None:
        plt.plot(path[:,0],path[:,1], 'g', linewidth=0.8)

    ax.set_aspect('equal')
    plt.show(block=True)



if not USE_ROS:
    if corridor2_world is None:
        plot_corridors(computed_path, corridor1_world)
    else:
        plot_corridors(computed_path, corridor1_world, corridor2_world)
    # plot_corridors(None, corridor1_world)
    # plot_corridors(None, corridor1_local, corridor2_local) 
    # plot_corridors(None, corridor1_world, corridor2_world)
    # plot_corridors(None, corridor1_converted, corridor2_converted, corridor1_world, corridor2_world)
    pass

else:
    if corridor2_converted is None:
        plot_corridors(computed_path, corridor1_converted)
    else:
        plot_corridors(computed_path, corridor1_converted, corridor2_converted)
        # plot_corridors(None, corridor1_converted)
        # plot_corridors(None, corridor1_local, corridor2_local) 
        # plot_corridors(None, corridor1_world, corridor2_world)
        # plot_corridors(None, corridor1_converted, corridor2_converted, corridor1_world, corridor2_world)

    exit()
    ##########################################################
    # Test 
    ##########################################################
    import rospy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, Vector3
    from barn_challenge.msg import maneuver

    def generate_path_message(input_path):
        path = Path() 
        if input_path.shape[0] == 0:
            return path

        for i in range(input_path.shape[0]):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = input_path[i,0]
            pose.pose.position.y = input_path[i,1] 
            pose.pose.position.z = 0
            pose.header.seq = path.header.seq + 1
            path.header.frame_id = "odom"
            path.header.stamp = rospy.Time.now()
            pose.header.stamp = path.header.stamp
            path.poses.append(pose)

        return path    

    def generate_maneuver_message(maneuver_array):
        maneuver_msg = maneuver()

        maneuver_msg.len = maneuver_array.shape[0]

        for i in range(maneuver_array.shape[0]):
            part_maneuver = Vector3()
            part_maneuver.x = maneuver_array[i,0]
            part_maneuver.y = maneuver_array[i,1]
            part_maneuver.z = maneuver_array[i,2]
            maneuver_msg.maneuver.append(part_maneuver)
        
        return maneuver_msg

    rospy.init_node('path_node', anonymous=True)
    rate = rospy.Rate(1) 

    path_Pub = rospy.Publisher('/path_corridors', Path, queue_size=1)

    maneuver_Pub = rospy.Publisher('/maneuver', maneuver, queue_size=1)


    while not rospy.is_shutdown():
        path_Pub.publish(generate_path_message(computed_path))
        maneuver_Pub.publish(generate_maneuver_message(sequence_man))
        rate.sleep()
