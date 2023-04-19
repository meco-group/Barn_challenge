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
from barn_challenge.msg import corridor_msg, corridor_world_msg, corridor_list
from corridor_helpers import *
from corridor_world import CorridorWorld
from visualization_msgs.msg import Marker, MarkerArray

class odomMsgClass():
    def __init__(self):
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0


def transform_corridor_to_world(new_corridor):
    '''
        Function takes the local corridor message (new_corridor) and creates a 
        CorridorWorld object from this
    '''
    
    L = np.sqrt(new_corridor.center_local[0]**2 +
                new_corridor.center_local[1]**2)
    phi = np.arctan2(new_corridor.center_local[1],new_corridor.center_local[0])\
        - np.pi/2 + new_corridor.init_pos_global[2]
    center = [new_corridor.init_pos_global[0] - L*np.sin(phi), 
              new_corridor.init_pos_global[1] + L*np.cos(phi)]
    
    tilt = -np.pi/2 + new_corridor.tilt_local + new_corridor.init_pos_global[2]

    L_growth = np.sqrt(new_corridor.growth_center_local[0]**2 + 
                       new_corridor.growth_center_local[1]**2)
    phi_growth = np.arctan2(new_corridor.growth_center_local[1], 
                            new_corridor.growth_center_local[0]) - np.pi/2 + \
                                new_corridor.init_pos_global[2]
    growth_center = [new_corridor.init_pos_global[0] - \
                        L_growth*np.sin(phi_growth), 
                     new_corridor.init_pos_global[1] + \
                        L_growth*np.cos(phi_growth)] 
    
    transformed_corridor = CorridorWorld(center=center, tilt=tilt, 
                                         height=new_corridor.height_local, 
                                         width=new_corridor.width_local)
    transformed_corridor.growth_center = growth_center
   
    return transformed_corridor

def process_new_corridor(new_corridor, curr_pose, root_corridor, 
                         current_corridor, explore_full_corridor):
    '''
        This function takes a new corridor message (new_corridor) and decides
        wether or not to put it in the corridor tree
    '''
    new_corridor = transform_corridor_to_world(new_corridor)

    # If this is the first corridor ever, start the tree
    if root_corridor is None:
        # root_corridor = new_corridor.corridor_deep_copy()
        root_corridor = new_corridor
        current_corridor = root_corridor
        print("[manager] Root corridor is now created")
        return (root_corridor, current_corridor)
    # Else, check if the new corridor should be added

    # If we are not yet in the current_corridor, this new corridor should be
    # added as a child to the parent of the current_corridor
    c = current_corridor
    if not explore_full_corridor and current_corridor is not None and \
        not current_corridor.check_inside(np.array([[curr_pose.posx], 
                                                    [curr_pose.posy], [1]])):
        c = current_corridor.parent

    # Check if this new corridor improves enough
    stuck = check_stuck(c, new_corridor, 0.25)
    if not stuck:
        print("[manager] Child corridor is added to the tree")
        c.add_child_corridor(new_corridor)
        c.remove_similar_children()
        c.sort_children()
    else:
        print("[manager] Discarding a corridor because it does not improve \
              enough")

    return (root_corridor, current_corridor)

def select_child_corridor(current_corridor):
    '''
        Based on the current corridor tree, this function selects a child to
        proceed
    '''
    if len(current_corridor.children) == 0:
        # backtrack
        return (False, current_corridor)
    else:
        current_corridor = current_corridor.children[0]
        return (True, current_corridor)

def corridorCallback(data):
    print("[manager] Starting corridor callback")
    new_message = corridor_msg()
    new_message.height_local = data.height_local
    new_message.width_local = data.width_local
    new_message.quality_local = data.quality_local
    new_message.center_local = data.center_local
    new_message.growth_center_local = data.growth_center_local
    new_message.tilt_local = data.tilt_local
    new_message.corners_local = data.corners_local
    new_message.init_pos_global = data.init_pos_global

    global new_corridor_present
    new_corridor_present = True

    global new_corridor_list
    new_corridor_list.append(new_message)

    # print("[manager] Corridor callback is executed!")
    # print("[manager] new_corridor_present = ", new_corridor_present)

def yaw_from_quaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))
def odomCallback(data):
    curr_pose.velx = data.twist.twist.linear.x
    curr_pose.rotz = data.twist.twist.angular.z
    curr_pose.posx = data.pose.pose.position.x
    curr_pose.posy = data.pose.pose.position.y
    curr_pose.theta = yaw_from_quaternion(data.pose.pose.orientation)

def publish_corridors(corridors, publisher):
    '''
        This function publishes the list of corridors so the navigator can use
        them
    '''
    if len(corridors) == 0:
        return
    
    to_send_list = corridor_list()
    for k in range(len(corridors)):
        to_send = corridor_world_msg()
        to_send.height_global = corridors[k].height
        to_send.width_global = corridors[k].width
        to_send.quality_global = corridors[k].quality
        to_send.center_global = corridors[k].center.copy()
        to_send.growth_center_global = corridors[k].growth_center.copy()
        to_send.tilt_global = corridors[k].tilt
        xy_corners = []
        for xy in corridors[k].corners:
            xy_corners.append(xy)
        to_send.corners_global = xy_corners.copy()

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
    '''
        Visualization for debugging purposes. The current branch along with
        other options is shown in different colors
    '''
    current_color = [1,0,0] # red
    branch_color  = [0,1,0] # green
    root_color  = [0,1,0]   # green
    options_color = [1,1,0]

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
    '''
        For debugging purposes, this function visualizes the backtracking
        corridors
    '''
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
    explore_full_corridor = True
    
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
                (root_corridor, current_corridor) = process_new_corridor(c, curr_pose, root_corridor, current_corridor, explore_full_corridor)
            
            new_corridor_present = False
            new_corridor_list = []

        # if we are manouvring in a tree, check if we can still proceed
        if current_corridor is not None:
            end_reached = check_end_of_corridor_reached(current_corridor, curr_pose, 0.7)
            if end_reached or (not explore_full_corridor and len(current_corridor.children) > 0):
                print("[manager] selecting child corridor")
                (succes, current_corridor) = select_child_corridor(current_corridor)
                print("[manager] child corridor selected! (", succes, ")")
                if not succes:
                    print("[manager] Need to backtrack!")
                    # pass the backtracking corridors to the motion planner
                    backtrack_mode_activated = True
                    (backtrack_point, current_corridor, backtracking_corridors) = get_back_track_point(current_corridor, explore_full_corridor)
                    if backtrack_point is None:
                        print("[manager] ERROR: I cannot backtrack because there are no other options")
                        backtrack_mode_activated = False
                    else:
                        publish_corridors(backtracking_corridors, corridor_pub)
                else:
                    publish_corridors([current_corridor], corridor_pub)

        if current_corridor is not None:
            visualize_corridor_tree(root_corridor, current_corridor)

        rate.sleep()


if __name__ == '__main__':
    main()
