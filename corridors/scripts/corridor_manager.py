#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math as m
import numpy as np
from barn_challenge.msg import corridor_msg
from corridor_helpers import *

def transformCorridorToWorld(newCorridor, curr_pose):
    newCorridor.center[0] += curr_pose.posx
    newCorridor.center[1] += curr_pose.posy
    newCorridor.growth_center[0] += curr_pose.posx
    newCorridor.growth_center[1] += curr_pose.posy
    newCorridor.corners = [[c[0]+curr_pose.posx, c[1]+curr_pose.posy] for c in newCorridor.corners]

    newCorridor.tilt = newCorridor.tilt + curr_pose.theta - np.pi/2

    newCorridor.update_W()

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
    new_corridor.new = True

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

def publishCurrentCorridor(current_corridor, publisher):
    to_send = corridor_msg()
    to_send.height = current_corridor.height
    to_send.width = current_corridor.width
    to_send.quality = current_corridor.quality
    to_send.center = current_corridor.center
    to_send.growth_center = current_corridor.growth_center
    to_send.tilt = current_corridor.tilt
    xy_corners = []
    for xy in current_corridor.corners_world:
        xy_corners.append(xy.x)
        xy_corners.append(xy.y)
    to_send.corners = xy_corners

    publisher.publish(to_send)

def main():

    # Subscribe to corridors published by corridor_fitter    
    global new_corridor
    new_corridor = corridor_msg()
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback)

    # Subscribe to odometry
    global curr_pose
    curr_pose = Odometry()
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)

    # Prepare to publish corridors
    corridor_pub = rospy.Publisher("/corridor", corridor_msg)

    rospy.init_node('manager', anonymous=True)
    rate = rospy.Rate(1)

    # Define variables for the corridor tree
    root_corridor = None
    current_corridor = None
    backtrack_point = None

    print('manager ready')
    while not rospy.is_shutdown():
        # if we are backtracking, just wait until we enter the new branch
        if backtrack_point is not None:
            if current_corridor.check_inside([[curr_pose.posx, curr_pose.posy]]):
                backtrack_point = None
            else:
                continue

        # if a new corridor arrived, potentially add it to the tree
        if new_corridor.new:
            processNewCorridor(new_corridor, curr_pose, root_corridor, current_corridor)
            new_corridor.new = False

        # if we are manouvring in a tree, check if we can still proceed
        if current_corridor is not None:
            end_reached = check_end_of_corridor_reached(current_corridor, curr_pose)
            if end_reached:
                succes = select_child_corridor(current_corridor)
                if not succes:
                    (backtrack_point, current_corridor, backtracking_corridors) = get_back_track_point(current_corridor)
                    # pass these corridors to the motion planner
                    # wait until we are at the backtrack point
                else:
                    corridor_pub.publish(current_corridor, corridor_pub)

        rate.sleep()


if __name__ == '__main__':
    main()
