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

class messageClass():
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0

def transformCorridorToWorld(newCorridor, currPos):
    # TODO: implement this function
    return newCorridor

def processNewCorridor(newCorridor, currPos, root_corridor, current_corridor):
    # NOTE: It is assumed that this corridor really is new
    newCorridor = transformCorridorToWorld(newCorridor, currPos)

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

def odomCallback(data):
    curr_pose = data

def main():
    global message
    message = messageClass()
    global new_corridor
    new_corridor = corridor_msg()
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback)

    global curr_pose
    curr_pose = pose_msg()
    pose_sub = rospy.Subscriber('/pose', pose_msg, odomCallback)

    # TODO: there should also be a subscription to the odometry topic

    rospy.init_node('manager', anonymous=True)
    rate = rospy.Rate(1)

    # Define variables for the corridor tree
    root_corridor = None
    current_corridor = None
    backtrack_point = None

    print('manager ready')
    while not rospy.is_shutdown():
        # if we are backtracking, just wait until we are close to the backtracking point
        if backtrack_point is not None:
            if np.power(curr_pose.x - backtrack_point[0], 2) + np.power(curr_pose.y - backtrack_point[1], 2) <= 0.5:
                backtrack_point = None
            else:
                pass

        # if a new corridor arrived, add it to the tree
        if new_corridor.new:
            processNewCorridor(new_corridor, curr_pose, root_corridor, current_corridor)
            new_corridor.new = False

        # if there exists some tree, manouvre in it
        if current_corridor is not None:
            end_reached = check_end_of_corridor_reached(current_corridor, curr_pose)
            if end_reached:
                succes = select_child_corridor(current_corridor)
                if not succes:
                    (backtrack_point, current_corridor, backtracking_corridors) = get_back_track_point(current_corridor)
                    # pass these corridors to the motion planner
                    # wait until we are at the backtrack point

        rate.sleep()


if __name__ == '__main__':
    main()
