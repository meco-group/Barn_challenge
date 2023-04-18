#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry, Path
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist, PoseStamped, Vector3
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor
from barn_challenge.msg import corridor_msg, corridor_list, maneuver

from motion_planner import compute_trajectory

class messageClass():
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


def yawFromQuaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))

def odomCallback(data):
    message.velx = data.twist.twist.linear.x
    message.rotz = data.twist.twist.angular.z
    message.posx = data.pose.pose.position.x
    message.posy = data.pose.pose.position.y
    message.theta = yawFromQuaternion(data.pose.pose.orientation)

# def corridorCallback(data):
#     b_corridor.height = data.height
#     b_corridor.width = data.width
#     b_corridor.quality = data.quality
#     b_corridor.center = data.center
#     b_corridor.tilt = data.tilt
#     b_corridor.corners = data.corners

def corridorListCallback(data):
    if data.len > 0:
        for corridor_message in data.corridors
            corridor_instance = Corridor(corridor_message.width, corridor_message.height, corridor_message.center, corridor_message.tilt + pi/2) 
            # Notice the addition of pi/2 on the corridor tilt
            list_of_corridors.append(corridor_instance)
    else:
        # TODO: If more than one corridor is sent, backtrack
        pass

def generate_path_message(input_path): # TODO: check that it works correctly with the world frame
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

def main():
    global message
    message = messageClass()
    global b_corridor
    b_corridor = corridor_msg()

    global list_of_corridors
    list_of_corridors = []

    # Subscribers
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    # corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback) # TODO: This subscriber is not needed (?)
    corridor_list_sub = rospy.Subscriber('/chosen_corridor', corridor_list, corridorListCallback)

    # Publishers
    path_Pub = rospy.Publisher('/path_corridors', Path, queue_size=1)
    maneuver_Pub = rospy.Publisher('/maneuver', maneuver, queue_size=1)

    rospy.init_node('navigator', anonymous=True)
    rate = rospy.Rate(100) # TODO: maybe this fast rate is not needed (due to the controller? node)

    # TODO: Check this control bounds
    v_max = 0.5
    v_min = -0.5
    omega_max = 1.57
    omega_min = -1.57
    u_bounds = np.array([v_min, v_max, omega_min, omega_max])
    a = 0.430
    b = 0.508
    m = 0.3

    # maxSpeed = 1
    # minSpeed = 0.1
    # maxTurn = m.pi/2
    isDone = False
    message.goalx = message.posx
    message.goaly = message.posy + 10

    while not rospy.is_shutdown():

        #####################################################
        # Compute path and maneuver within corridors
        #####################################################

        # Generate corridor instances from corridor messages
        # corridor1 = Corridor(width1, height1, center1_world, tilt1_world + pi/2) # The variables defining the corridors are just placeholders
        # corridor2 = Corridor(width2, height2, center2_world, tilt2_world + pi/2) # Notice the addition of pi/2
        
        if len(list_of_corridors) > 0:

            corridor1 = list_of_corridors[0]
            corridor2 = list_of_corridors[1] if len(list_of_corridors) > 1 else None

            # TODO: implement the logic on removing first corridor from list (.pop(0)) and 
            # shifting the corridors. First corridor should always be the one where the robot is.
            # Maybe use check_inside_one_point() from motion_planner.py

            # Compute the maneuvers within the corridors
            computed_maneuver, computed_path = compute_trajectory(
                corridor1 = corridor1, 
                u_bounds = u_bounds, 
                a = a, 
                b = b, 
                m = m, 
                x0 = message.posx, 
                y0 = message.posy, 
                theta0 = message.theta + m.pi/2, # TODO: Check this
                plot = False, 
                corridor2 = corridor2
            )

            # The computed maneuver should be sent to the controller, which will 
            # define the instantaneous twist to be sent to the robot
            maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
            # Publish computed path for visualization on RViz
            path_Pub.publish(generate_path_message(computed_path))

        # Finish execution if goal has been reached
        if isDone:
            rospy.signal_shutdown('Goal Reached')

        rate.sleep()


if __name__ == '__main__':
    main()
