#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan and louis
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math as m
import numpy as np
from barn_challenge.msg import corridor_msg, corridor_list
from corridor_helpers import *

class odomMsgClass():
    def __init__(self):
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0

def transformCorridorToWorld(newCorridor, curr_pose):
    # translate the center
    newCorridor.center[0] += curr_pose.posx
    newCorridor.center[1] += curr_pose.posy

    # rotate the tilt
    rotation_angle = curr_pose.theta - np.pi/2
    newCorridor.tilt = newCorridor.tilt + rotation_angle

    # translate and rotate the growth center
    a = newCorridor.growth_center[0] + curr_pose.posx
    b = newCorridor.growth_center[1] + curr_pose.posy
    c = a*np.cos(rotation_angle)-b*np.sin(rotation_angle)
    d = b*np.cos(rotation_angle)+a*np.sin(rotation_angle)
    newCorridor.growth_center = [c, d]

    # update W and the corners
    newCorridor.update_W()
    newCorridor.corners = newCorridor.getCorners()

    return newCorridor

def processNewCorridor(newCorridor, curr_pose, root_corridor, current_corridor):
    # NOTE: It is assumed that this corridor really is new
    newCorridor = transformCorridorToWorld(newCorridor, curr_pose)

    # If this is the first corridor ever, start the tree
    if root_corridor is None:
        root_corridor = newCorridor.corridor_deep_copy()
        current_corridor = root_corridor
        return
    # Else, check if the new corridor should be added

    stuck = check_stuck(current_corridor, newCorridor, 0.25)
    if not stuck:
        current_corridor.add_child(newCorridor)
        current_corridor.remove_similar_children()

def select_child_corridor(current_corridor):
    if len(select_child_corridor.children) == 0:
        # backtrack
        return False
    else:
        current_corridor = current_corridor.children[0]
        return True

def corridorCallback(data):
    new_corridor.height = data.height
    new_corridor.width = data.width
    new_corridor.quality = data.quality
    new_corridor.center = data.center
    new_corridor.tilt = data.tilt
    new_corridor.corners = data.corners
    new_corridor_present = True

def yawFromQuaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))
def odomCallback(data):
    curr_pose.velx = data.twist.twist.linear.x
    curr_pose.rotz = data.twist.twist.angular.z
    curr_pose.posx = data.pose.pose.position.x
    curr_pose.posy = data.pose.pose.position.y
    curr_pose.theta = yawFromQuaternion(data.pose.pose.orientation)

def publishCorridors(corridors, publisher):
    if len(corridors) == 0:
        return
    
    to_send_list = corridor_list()
    for k in range(len(corridors)):
        to_send = corridor_msg()
        to_send.height = corridors.height
        to_send.width = corridors.width
        to_send.quality = corridors.quality
        to_send.center = corridors.center
        to_send.growth_center = corridors.growth_center
        to_send.tilt = corridors.tilt
        xy_corners = []
        for xy in corridors.corners_world:
            xy_corners.append(xy.x)
            xy_corners.append(xy.y)
        to_send.corners = xy_corners

        to_send_list.len += 1
        to_send_list.corridor_msg.append(to_send)

    publisher.publish(to_send_list)

def main():

    # Subscribe to corridors published by corridor_fitter    
    global new_corridor
    global new_corridor_present
    new_corridor_present = False
    new_corridor = corridor_msg()
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback)

    # Subscribe to odometry
    global curr_pose
    curr_pose = odomMsgClass()
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)

    # Prepare to publish corridors
    corridor_pub = rospy.Publisher("/corridor", corridor_list)

    rospy.init_node('manager', anonymous=True)
    rate = rospy.Rate(1)

    # Define variables for the corridor tree
    root_corridor = None
    current_corridor = None
    backtrack_point = None
    backtrack_mode_activated = False

    print('manager ready')
    while not rospy.is_shutdown():
        print("Manager looping")
        # if we are backtracking, just wait until we enter the new branch
        if backtrack_mode_activated:
            if current_corridor.check_inside([[curr_pose.posx, curr_pose.posy]]):
                backtrack_mode_activated = False
            else:
                continue

        # if a new corridor arrived, potentially add it to the tree
        if new_corridor_present:
            processNewCorridor(new_corridor, curr_pose, root_corridor, current_corridor)
            new_corridor_present = False

        # if we are manouvring in a tree, check if we can still proceed
        if current_corridor is not None:
            end_reached = check_end_of_corridor_reached(current_corridor, curr_pose)
            if end_reached:
                succes = select_child_corridor(current_corridor)
                if not succes:
                    # pass the backtracking corridors to the motion planner
                    backtrack_mode_activated = True
                    (backtrack_point, current_corridor, backtracking_corridors) = get_back_track_point(current_corridor)
                    publishCorridors(backtracking_corridors, corridor_pub)
                else:
                    publishCorridors([current_corridor], corridor_pub)

        rate.sleep()


if __name__ == '__main__':
    main()
