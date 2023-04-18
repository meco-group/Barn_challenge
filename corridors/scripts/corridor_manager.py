#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan and louis
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math as m
import numpy as np
from barn_challenge.msg import corridor_msg, corridor_list
from corridor_helpers import *
from corridor import Corridor
from visualization_msgs.msg import Marker, MarkerArray

class odomMsgClass():
    def __init__(self):
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0

### OLD ###
# def transformCorridorToWorld(newCorridor, curr_pose):
#     # translate the center
#     center = [newCorridor.center[0] + curr_pose.posx, newCorridor.center[1] + curr_pose.posy]

#     # rotate the tilt
#     rotation_angle = curr_pose.theta - np.pi/2
#     tilt = newCorridor.tilt + rotation_angle

#     # translate and rotate the growth center
#     a = newCorridor.growth_center[0] + curr_pose.posx
#     b = newCorridor.growth_center[1] + curr_pose.posy
#     c = a*np.cos(rotation_angle)-b*np.sin(rotation_angle)
#     d = b*np.cos(rotation_angle)+a*np.sin(rotation_angle)
#     growth_center = [c, d]

#     transformed_corridor = Corridor(center=center, height=newCorridor.height, width=newCorridor.width)
#     transformed_corridor.growth_center = growth_center

#     # update W and the corners
#     transformed_corridor.update_W()
#     transformed_corridor.corners = get_corners(transformed_corridor.W)

#     return transformed_corridor

def transformCorridorToWorld(newCorridor):
    # Correct transformation by Alejandro
    
    print('corr init pos', newCorridor.init_pos)
    L = np.sqrt(newCorridor.center[0]**2 + newCorridor.center[1]**2)
    phi = np.arctan2(newCorridor.center[1],newCorridor.center[0]) - np.pi/2 + newCorridor.init_pos[2]
    center = [newCorridor.init_pos[0] - L*np.sin(phi), newCorridor.init_pos[1] + L*np.cos(phi)]
    
    tilt = newCorridor.tilt + newCorridor.init_pos[2]

    L_growth = np.sqrt(newCorridor.growth_center[0]**2 + newCorridor.growth_center[1]**2)
    phi_growth = np.arctan2(newCorridor.growth_center[1],newCorridor.growth_center[0]) - np.pi/2 + newCorridor.init_pos[2]
    growth_center = [newCorridor.init_pos[0] - L_growth*np.sin(phi_growth), newCorridor.init_pos[1] + L_growth*np.cos(phi_growth)] 
    
    transformed_corridor = Corridor(center=center, tilt=tilt, height=newCorridor.height, width=newCorridor.width)
    transformed_corridor.growth_center = growth_center
    
    transformed_corridor.update_W()
    transformed_corridor.corners_world = transformed_corridor.get_corners().copy()
    
    return transformed_corridor

def processNewCorridor(newCorridor, curr_pose, root_corridor, current_corridor, EXPLORE_FULL_CORRIDOR):
    # NOTE: It is assumed that this corridor really is new
    newCorridor = transformCorridorToWorld(newCorridor)

    # If this is the first corridor ever, start the tree
    if root_corridor is None:
        # root_corridor = newCorridor.corridor_deep_copy()
        root_corridor = newCorridor
        current_corridor = root_corridor
        print("[manager] Root corridor is now created")
        return (root_corridor, current_corridor)
    # Else, check if the new corridor should be added

    c = current_corridor
    if EXPLORE_FULL_CORRIDOR and not current_corridor.check_inside(np.array([[curr_pose.posx], [curr_pose.posy], [1]])):
        c = current_corridor.parent

    stuck = check_stuck(c, newCorridor, 0.25)
    if not stuck:
        print("[manager] Child corridor is added to the tree")
        c.add_child_corridor(newCorridor)
        c.remove_similar_children()
        c.sort_children()
    else:
        print("[manager] Discarding a corridor because it does not improve enough")

    return (root_corridor, current_corridor)

def select_child_corridor(current_corridor):
    if len(current_corridor.children) == 0:
        # backtrack
        return (False, current_corridor)
    else:
        current_corridor = current_corridor.children[0]
        return (True, current_corridor)

def corridorCallback(data):
    print("[manager] Starting corridor callback")
    new_message = corridor_msg()
    new_message.height = data.height
    new_message.width = data.width
    new_message.quality = data.quality
    new_message.center = data.center
    new_message.growth_center = data.growth_center
    new_message.tilt = data.tilt
    new_message.corners = data.corners
    new_message.init_pos = data.init_pos

    global new_corridor_present
    new_corridor_present = True

    global new_corridor_list
    new_corridor_list.append(new_message)

    # print("[manager] Corridor callback is executed!")
    # print("[manager] new_corridor_present = ", new_corridor_present)

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
        to_send.height = corridors[k].height
        to_send.width = corridors[k].width
        to_send.quality = corridors[k].quality
        to_send.center = corridors[k].center.copy()
        to_send.growth_center = corridors[k].growth_center.copy()
        to_send.tilt = corridors[k].tilt
        xy_corners = []
        for xy in corridors[k].corners:
            xy_corners.append(xy)
        to_send.corners = xy_corners.copy()

        to_send_list.len += 1
        to_send_list.corridors.append(to_send)

    print("[manager] Published ", to_send_list.len, " corridor(s)")
    publisher.publish(to_send_list)

def visualize_rectangle(rect, i, r, g, b):
    '''Visualize corridors.
    '''
    corners = []
    for k in range(len(rect)):
        corners.append(Point(rect[k][0], rect[k][1], 0))

    rect_marker = Marker()
    rect_marker.header.stamp = rospy.Time.now()
    rect_marker.header.frame_id = 'odom'
    rect_marker.ns = 'rect'
    rect_marker.id = i
    rect_marker.action = 0
    rect_marker.scale.x = 0.03
    rect_marker.color.r = r
    rect_marker.color.g = g
    rect_marker.color.b = b
    rect_marker.color.a = 1.0
    rect_marker.pose.orientation.w = 1
    # rect_marker.lifetime = rospy.Duration(0)
    rect_marker.lifetime = rospy.Time(1)
    rect_marker.type = 4  # Line Strip
    rect_marker.points = corners + [corners[0]]

    name = '/rect'
    marker_pub = rospy.Publisher(name, MarkerArray, queue_size=10)
    marker_arr = MarkerArray()

    marker_arr.markers.append(rect_marker)
    marker_pub.publish(marker_arr)

def visualize_corridor_tree(root_corridor, current_corridor):
    current_color = [1,0,0] # blue
    branch_color  = [0,1,0] # white
    root_color  = [0,1,0]   # white
    options_color = [1,1,0] # optional branches

    global id

    # visualize current corridor
    visualize_rectangle(current_corridor.corners, id, current_color[0], current_color[1], current_color[2])
    id += 1
    
    # visualize current branch and all other children
    to_visualize = current_corridor
    while to_visualize.parent is not None:
        # print("[manager] looping visualization (", current_corridor, ")")
        to_visualize = current_corridor.parent
        visualize_rectangle(to_visualize.corners, id, branch_color[0], branch_color[1], branch_color[2])
        id += 1

        for k in range(1,len(to_visualize.children)):
            visualize_rectangle(to_visualize.children[k].corners, id, options_color[0], options_color[1], options_color[2])
            id += 1

    # visualize root corridor
    visualize_rectangle(root_corridor.corners, id, root_color[0], root_color[1], root_color[2])
    id += 1

    return

def visualize_backtracking_corridors(backtracking_corridors, current_corridor):
    backtrack_color = [0.6,0.6,0.6] # dark grey
    
    global id
    for c in backtracking_corridors:
        visualize_rectangle(c.corners, id, backtrack_color[0], backtrack_color[1], backtrack_color[2])
        id += 1

    visualize_rectangle(current_corridor.corners, id, 0,0,1)
    id += 1

    return


def main():

    global id # visualization id
    id = 0

    # If set to True, a child corridor will only be when the end of the current corridor is reached
    # If set to False, a child is selected as soon as its available, should only be used for debugging
    EXPLORE_FULL_CORRIDOR = True
    
    # Subscribe to incoming new corridors
    global new_corridor_list
    new_corridor_list = []

    global new_corridor_present
    new_corridor_present = False
    
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback)

    # Subscribe to odometry
    global curr_pose
    curr_pose = odomMsgClass()
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)

    # Prepare to publish corridors
    corridor_pub = rospy.Publisher("/chosen_corridor", corridor_list)

    rospy.init_node('manager', anonymous=True)
    rate = rospy.Rate(1)

    # Define variables for the corridor tree
    root_corridor = None
    current_corridor = None
    backtrack_point = None
    backtrack_mode_activated = False

    print('[manager] manager ready')
    while not rospy.is_shutdown():
        print("[manager] Manager looping")

        # if we are backtracking, just wait until we enter the new branch
        if backtrack_mode_activated:
            print("[manager] Backtracking...")
            visualize_backtracking_corridors(backtracking_corridors, current_corridor)
            if current_corridor.check_inside(np.array([[curr_pose.posx], [curr_pose.posy], [1]])):
                backtrack_mode_activated = False
            else:
                rate.sleep()
                continue

        # if a new corridor arrived, potentially add it to the tree
        if new_corridor_present:
            print("[manager] discovered ",len(new_corridor_list), " new corridor(s)!")

            for c in new_corridor_list:
                # print("[manager] processing corridor (", c.width, ", ", c.height, ", ", c.center, ")")
                (root_corridor, current_corridor) = processNewCorridor(c, curr_pose, root_corridor, current_corridor, EXPLORE_FULL_CORRIDOR)
            
            new_corridor_present = False
            new_corridor_list = []

        # if we are manouvring in a tree, check if we can still proceed
        if current_corridor is not None:
            end_reached = check_end_of_corridor_reached(current_corridor, curr_pose, 0.7)
            if end_reached or (not EXPLORE_FULL_CORRIDOR and len(current_corridor.children) > 0):
                print("[manager] selecting child corridor")
                (succes, current_corridor) = select_child_corridor(current_corridor)
                print("[manager] child corridor selected! (", succes, ")")
                if not succes:
                    print("[manager] Need to backtrack!")
                    # pass the backtracking corridors to the motion planner
                    backtrack_mode_activated = True
                    (backtrack_point, current_corridor, backtracking_corridors) = get_back_track_point(current_corridor, EXPLORE_FULL_CORRIDOR)
                    if backtrack_point is None:
                        print("[manager] ERROR: I cannot backtrack because there are no other options")
                        backtrack_mode_activated = False
                    else:
                        publishCorridors(backtracking_corridors, corridor_pub)
                else:
                    publishCorridors([current_corridor], corridor_pub)

        if current_corridor is not None:
            visualize_corridor_tree(root_corridor, current_corridor)

        rate.sleep()


if __name__ == '__main__':
    main()
