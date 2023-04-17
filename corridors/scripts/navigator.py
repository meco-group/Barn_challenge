#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry, Path
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist, PoseStamped
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor
from barn_challenge.msg import corridor_msg
from barn_challenge.msg import corridor_list

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

def corridorCallback(data):
    b_corridor.height = data.height
    b_corridor.width = data.width
    b_corridor.quality = data.quality
    b_corridor.center = data.center
    b_corridor.tilt = data.tilt
    b_corridor.corners = data.corners

def corridorListCallback(data):
    if len(data) > 0:
        best_corridor = data[0]
        b_corridor.height = best_corridor.height
        b_corridor.width = best_corridor.width
        b_corridor.quality = best_corridor.quality
        b_corridor.center = best_corridor.center
        b_corridor.tilt = best_corridor.tilt
        b_corridor.corners = best_corridor.corners
    else:
        # TODO: If more than one corridor is sent, backtrack
        pass

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


def main():
    global message
    message = messageClass()
    global b_corridor
    b_corridor = corridor_msg()

    # Subscribers
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback) # TODO: This subscriber is not needed (?)
    corridor_list_sub = rospy.Subscriber('/chosen_corridor', corridor_list, corridorListCallback)

    # Publishers
    vel_Pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                              queue_size=10) # TODO:  Should a maneuver publisher replace this?
    path_Pub = rospy.Publisher('/path_corridors', Path, queue_size=1)

    rospy.init_node('navigator', anonymous=True)
    rate = rospy.Rate(100)

    # TODO: Check this control bounds
    v_max = 2
    v_min = -2
    omega_max = 2
    omega_min = -2
    u_bounds = np.array([v_min, v_max, omega_min, omega_max])
    a = 0.430
    b = 0.508
    m = 0.3

    maxSpeed = 1
    minSpeed = 0.1
    maxTurn = m.pi/2
    isDone = False
    twist = Twist()
    twist.linear.x = 0.
    twist.angular.z = 0.
    message.goalx = message.posx
    message.goaly = message.posy + 10

    print('I started moving')
    while not rospy.is_shutdown():


        print('current_position', message.posx, message.posy, message.theta)

        #####################################################
        # Compute path and maneuver within corridors
        #####################################################

        # Generate corridor instances from corridor messages
        corridor1 = Corridor(width1, height1, center1_world, tilt1_world + pi/2) # TODO: The variables defining the corridors are just placeholders
        corridor2 = Corridor(width2, height2, center2_world, tilt2_world + pi/2) # Notice the addition of pi/2

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

        # Start moving slowly.
        twist.linear.x = 0.
        twist.angular.z = 0.

        distToGoal = distance((message.goalx, message.goaly),
                              (message.posx, message.posy))

        if distToGoal < 0.5:
            twist.linear.x = 0.
            twist.angular.z = 0.
            print('I arrived')
            isDone = True

        #####################################################
        # Node outputs
        #####################################################
        vel_Pub.publish(twist) # This should go in a controller node (replace with maneuver publisher)
        # Publish the computed path for debugging
        path_Pub.publish(generate_path_message(computed_path))

        # Finish execution if goal has been reached
        if isDone:
            rospy.signal_shutdown('Goal Reached')

        rate.sleep()


if __name__ == '__main__':
    main()
