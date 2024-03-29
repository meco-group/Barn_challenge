#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan and louis
"""

import rospy
from geometry_msgs.msg import Pose2D
import math as m
import numpy as np
from barn_challenge.msg import \
    CorridorLocalMsg, CorridorLocalListMsg, \
    CorridorWorldMsg, CorridorWorldListMsg, \
    GoalMsg
from corridor_helpers import *
from corridor_world import CorridorWorld
from motion_planner import check_inside_one_point
from visualization_msgs.msg import Marker


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
    CorridorWorld object from this.
    '''

    L = np.sqrt(new_corridor.center_local[0]**2 +
                new_corridor.center_local[1]**2)
    phi = np.arctan2(new_corridor.center_local[1],
                     new_corridor.center_local[0]) \
        - np.pi/2 + new_corridor.init_pos_global[2]

    center = [new_corridor.init_pos_global[0] - L*np.sin(phi),
              new_corridor.init_pos_global[1] + L*np.cos(phi)]

    tilt = -np.pi/2 + new_corridor.tilt_local + new_corridor.init_pos_global[2]

    L_growth = np.sqrt(new_corridor.growth_center_local[0]**2 +
                       new_corridor.growth_center_local[1]**2)
    phi_growth = np.arctan2(new_corridor.growth_center_local[1],
                            new_corridor.growth_center_local[0]) - np.pi/2 + \
        new_corridor.init_pos_global[2]
    growth_center = [new_corridor.init_pos_global[0] -
                     L_growth*np.sin(phi_growth),
                     new_corridor.init_pos_global[1] +
                     L_growth*np.cos(phi_growth)]

    transformed_corridor = CorridorWorld(center=center, tilt=tilt,
                                         height=new_corridor.height_local,
                                         width=new_corridor.width_local)

    transformed_corridor.growth_center = growth_center

    return transformed_corridor


def process_new_corridor(new_corridor_msg, root_corridor,
                         current_corridor, orphanage,
                         explore_full_corridor, corridor_pub,
                         backtrack_mode_activated):
    '''
    This function takes a new corridor message (new_corridor) and decides
    wether or not to put it in the corridor tree.
    '''
    new_corridor = transform_corridor_to_world(new_corridor_msg)

    # If this is the first corridor ever, start the tree
    if root_corridor is None:
        # root_corridor = new_corridor.corridor_deep_copy()
        root_corridor = new_corridor
        current_corridor = root_corridor
        # print("[manager] Root corridor is now created")
        publish_corridors([root_corridor], corridor_pub,
                          backtrack_mode_activated)
        return (root_corridor, current_corridor)
    # Else, check if the new corridor should be added

    # If we are not yet in the current_corridor, this new corridor should be
    # added as a child to the parent of the current_corridor
    receiving_corridor = current_corridor
    if not explore_full_corridor and current_corridor is not None and \
        current_corridor.parent is not None and \
        not current_corridor.check_inside(np.array(
            [[new_corridor_msg.init_pos_global[0]],
             [new_corridor_msg.init_pos_global[1]],
             [1]])):
        return (root_corridor, current_corridor)
        # receiving_corridor = current_corridor.parent

    # Check if goal position is reachable
    global rotated_goal
    if check_inside_one_point(new_corridor, rotated_goal):
        # print("[manager] Child corridor is added to the tree")
        receiving_corridor.add_child_corridor(new_corridor)
        receiving_corridor.remove_similar_children()
        receiving_corridor.sort_children()
        # Don't know why, but this line must be here
        global GOAL_IN_SIGHT
        GOAL_IN_SIGHT = True
        print("*************************************")
        print("[manager] Goal is in sight!")
        publish_final_corridor(new_corridor, corridor_pub)
    else:
        # Check if this new corridor improves enough
        (stuck, d) = check_stuck(receiving_corridor, new_corridor)

        # Check if this new corridor is not too similar to a
        # backtracked corridor
        backtracked = orphanage.has_similar_child(new_corridor, distance_threshold=2.0, tilt_threshold=0)

        if not stuck and not backtracked:
            # print("[manager] Child corridor added")
            receiving_corridor.add_child_corridor(new_corridor, margin=.2)
            receiving_corridor.sort_children()
            receiving_corridor.remove_similar_children()
        # elif stuck:
            # print(f"[manager] Discarding a corridor (stuck: {round(d,3)})")
        # else:
            # print("[manager] Discarding a corridor (backtrack similarity)")

    return (root_corridor, current_corridor)


def select_child_corridor(current_corridor):
    '''
    Based on the current corridor tree, this function selects a child to
    proceed
    It returns a boolean to indicate if a child was succesfully selected and
    also returns the (potentially new) current corridor
    '''
    if len(current_corridor.children) == 0:
        # backtrack
        return (False, current_corridor)
    else:
        if not current_corridor.has_quality_child():
            return (True, current_corridor.children[0])

        else:
            for child in current_corridor.children:
                if child.quality >= 0.5:
                    return (True, child)

            # print("[manager] ERROR: select_child_corridor")
            return (False, current_corridor)


def corridorCallback(data):
    # print("[manager] Starting corridor callback")
    global new_corridor_present
    new_corridor_present = True
    global new_corridor_list

    for i in range(data.len):
        new_message = CorridorLocalMsg()
        new_message.height_local = data.corridors[i].height_local
        new_message.width_local = data.corridors[i].width_local
        new_message.quality_local = data.corridors[i].quality_local
        new_message.center_local = data.corridors[i].center_local
        new_message.growth_center_local = data.corridors[i].growth_center_local
        new_message.tilt_local = data.corridors[i].tilt_local
        new_message.corners_local = data.corridors[i].corners_local
        new_message.init_pos_global = data.corridors[i].init_pos_global

        new_corridor_list.append(new_message)


def yaw_from_quaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))


def odomCallback(data):
    curr_pose.posx = data.x
    curr_pose.posy = data.y
    curr_pose.theta = data.theta


def publish_corridors(corridors, publisher, backtrack_mode_activated):
    '''
    This function publishes the list of corridors so the navigator can use
    them.
    '''
    if backtrack_mode_activated:
        print("\n[MANAGER WARNING]")
        print("Illegal attempt to publish a corridor!\n")
        return

    if len(corridors) == 0:
        return

    to_send_list = CorridorWorldListMsg()
    for corridor in corridors:
        to_send = CorridorWorldMsg()
        to_send.height_global = corridor.height
        to_send.width_global = corridor.width
        to_send.quality_global = corridor.quality
        to_send.center_global = corridor.center.copy()
        to_send.growth_center_global = corridor.growth_center.copy()
        to_send.tilt_global = corridor.tilt
        xy_corners = []
        for xy in corridor.corners:
            xy_corners.append(list(xy))
        # print(xy_corners)
        to_send.corners_global = []
        to_send.goal_in_sight = False

        to_send_list.len += 1
        to_send_list.corridors.append(to_send)

    # print("[manager] Published ", to_send_list.len, " corridor(s)")
    publisher.publish(to_send_list)

def publish_final_corridor(final_corridor, publisher):
    '''
    This function publishes the final corridors so the navigator can use
    them.
    '''

    to_send_list = CorridorWorldListMsg()

    to_send = CorridorWorldMsg()
    to_send.height_global = final_corridor.height
    to_send.width_global = final_corridor.width
    to_send.quality_global = final_corridor.quality
    to_send.center_global = final_corridor.center.copy()
    to_send.growth_center_global = final_corridor.growth_center.copy()
    to_send.tilt_global = final_corridor.tilt
    xy_corners = []
    for xy in final_corridor.corners:
        xy_corners.append(list(xy))
    # print(xy_corners)
    to_send.corners_global = []
    to_send.goal_in_sight = True

    to_send_list.len = 1
    to_send_list.corridors = [to_send]

    # print("[manager] Published final corridor")
    publisher.publish(to_send_list)

def rviz_visualization_goal(x, y, z):
    '''Visualize corridors.
    '''
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = 'map'
    marker.ns = 'goal2'
    marker.id = 1
    marker.action = 0
    marker.scale.x = .1
    marker.scale.y = .1
    marker.scale.z = .1
    marker.color.r = 1.
    marker.color.g = 1.
    marker.color.b = 1.
    marker.color.a = 1.0
    marker.lifetime = rospy.Time(1.)
    marker.type = marker.SPHERE
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1

    name = '/' + marker.ns
    marker_pub = rospy.Publisher(name, Marker, queue_size=1)
    marker_pub.publish(marker)


def visualize_corridor_tree(root_corridor, current_corridor, margin):
    '''
    Visualization for debugging purposes. The current branch along with
    other options is shown in different colors
    '''
    global id
    margin_color = [1, 0.6, 0.6]  # light red
    current_color = [1, 0, 0]  # red
    branch_color = [0, 1, 0]   # green
    options_color = [1, 1, 0]  # yellow
    current_children_color = [1, 0.5, 0]  # orange

    # visualize current corridor
    current_corridor.rviz_visualization('rect', id, *current_color,
                                        1/manager_rate)

    id += 1
    # corridor_margin = CorridorWorld(center=current_corridor.center,
    #                                 tilt=current_corridor.tilt,
    #                                 height=current_corridor.height-2*margin,
    #                                 width=current_corridor.width-2*margin)
    # corridor_margin.rviz_visualization('rect', id, *margin_color,
    #                                    1/manager_rate)
    # id += 1

    for child in current_corridor.children:
        child.rviz_visualization('rect', id, *current_children_color,
                                 1/manager_rate)
        id += 1

    # # visualize current branch and all other children
    # to_visualize = current_corridor
    # while to_visualize.parent is not None:
    #     to_visualize = to_visualize.parent
    #     to_visualize.rviz_visualization('rect', id, *branch_color,
    #                                     1/manager_rate)
    #     id += 1

    #     for k in range(1, len(to_visualize.children)):
    #         to_visualize.children[k].rviz_visualization('rect', id,
    #                                                     *options_color,
    #                                                     1/manager_rate)
    #         id += 1


def visualize_backtracking_corridors(backtracking_corridors, current_corridor):
    '''
    For debugging purposes, this function visualizes the backtracking
    corridors
    '''
    global id
    backtrack_color = [1, 0, 1]  # purple
    current_color = [1, 0, 0]  # red

    for corridor in backtracking_corridors:
        corridor.rviz_visualization('rect', id, *backtrack_color,
                                    1/manager_rate)
        id += 1

    current_corridor.rviz_visualization('rect', id, *current_color,
                                        1/manager_rate)
    id += 1


def main():
    global id  # visualization id
    id = 0

    global manager_rate
    manager_rate = 10

    global GOAL_IN_SIGHT
    GOAL_IN_SIGHT = False

    # If set to True, a child corridor will only be selected when the end of
    # the current corridor is reached.
    # If set to False, a child is selected as soon as its available, should
    # only be used for debugging.
    explore_full_corridor = False

    # Subscribe to incoming new corridors
    global new_corridor_list
    new_corridor_list = []

    global new_corridor_present
    new_corridor_present = True

    corridor_sub = rospy.Subscriber('/corridor', CorridorLocalListMsg,
                                    corridorCallback)

    # Subscribe to odometry
    global curr_pose
    curr_pose = odomMsgClass()
    odom_sub = rospy.Subscriber('/pose_map', Pose2D, odomCallback)

    # Prepare to publish corridors
    corridor_pub = rospy.Publisher("/chosen_corridor", CorridorWorldListMsg, queue_size=10)

    rospy.init_node('manager', anonymous=True)
    rate = rospy.Rate(manager_rate)

    # Define variables for the corridor tree
    root_corridor = None
    current_corridor = None
    orphanage = CorridorWorld(center=[0.0, 0.0], width=0.001,
                              height=0.001, tilt=0.0)

    backtrack_point = None
    waiting_to_backtrack = False
    backtrack_mode_activated = False
    backtrack_waiting_time = 1.5
    backtrack_start_time = None
    backtrack_curr_time = None

    waiting_at_end_reached = False
    end_reached_waiting_time = 0.5
    end_reached_start_time = None
    end_reached_curr_time = None

    # almost ready to go, but first publish the GOAL POSITION
    # considering the initial tilt of the robot
    global rotated_goal
    goal = [0.0, 10.0]
    rotation_angle = curr_pose.theta - np.pi/2
    rotated_goal = [np.cos(rotation_angle)*goal[0] - np.sin(rotation_angle)*goal[1],
                    np.sin(rotation_angle)*goal[0] + np.cos(rotation_angle)*goal[1]]
    goal_pub = rospy.Publisher("/goal_position", GoalMsg, queue_size=10)

    rviz_visualization_goal(*rotated_goal, 0.)

    rotated_goal_msg = GoalMsg()
    rotated_goal_msg.goal = rotated_goal
    goal_pub.publish(rotated_goal_msg)

    print('[manager] Manager ready')
    while not rospy.is_shutdown():
        # If goal in sight, there is no more work to do
        goal_pub.publish(rotated_goal_msg)
        rviz_visualization_goal(*rotated_goal, 0.)

        if GOAL_IN_SIGHT:
            # print("[manager] Goal in sight -- stopping")
            visualize_corridor_tree(root_corridor, current_corridor, margin=0.1)
            rate.sleep()
            continue

        # if we are backtracking, just wait until we enter the new branch
        if backtrack_mode_activated:
            height = current_corridor.height
            m = .1  # from navigator TODO
            tilt = current_corridor.tilt
            backtracking_goal = current_corridor.center + \
                np.array([(height/2 - m)*np.sin(tilt),
                         -(height/2 - m)*np.cos(tilt)])
            # print("[manager] Backtracking...")
            visualize_backtracking_corridors(backtracking_corridors,
                                             current_corridor)
            if check_inside_one_point(current_corridor,
                                      [[curr_pose.posx], [curr_pose.posy]]):
                # np.sqrt((curr_pose.posx - backtracking_goal[0])**2 +
                #        (curr_pose.posy - backtracking_goal[1])**2) < .5:
                # print("[manager] Stopping backtracking!")
                backtrack_mode_activated = False
                # Flush all corridors heard while backtracking
                new_corridor_list = []
                new_corridor_present = False
                # publish_corridors([current_corridor], corridor_pub, backtrack_mode_activated)
            else:
                rate.sleep()
                continue

        # if a new corridor arrived while we are not backtracking,
        # check to add it to the tree
        if not backtrack_mode_activated and new_corridor_present:
            # print("[manager] discovered ", len(new_corridor_list),
            #       " new corridor(s)!")

            for corridor in new_corridor_list:
                (root_corridor, current_corridor) = process_new_corridor(
                    corridor, root_corridor, current_corridor, orphanage,
                    explore_full_corridor, corridor_pub,
                    backtrack_mode_activated)

            new_corridor_present = False
            new_corridor_list = []

        # if we are manouvring forward in a tree, check if we can still proceed
        if not backtrack_mode_activated and current_corridor is not None:
            
            end_reached = check_end_of_corridor_reached(
                current_corridor, curr_pose, 0.5)
            current_corridor.fully_explored = end_reached

            # select a child corridor if
            #   - you have reached the end of the current corridor
            #   - you don't want to explore the full corridor and you
            #       know you already have a child with a satisfactory quality
            if end_reached or (not explore_full_corridor and
                               current_corridor.has_quality_child()):
                
                # if you arrive at the end, wait a bit, otherwise go for the
                # high quality child
                succes = False
                if end_reached and not waiting_at_end_reached: # you were not waiting yet
                    waiting_at_end_reached = True
                    end_reached_start_time = rospy.Time.now().to_sec()
                elif end_reached and waiting_at_end_reached:   # you were already waiting
                    end_reached_curr_time = rospy.Time.now().to_sec()
                    if (end_reached_curr_time-end_reached_start_time > 
                        end_reached_waiting_time):
                        waiting_at_end_reached = False
                        (succes, current_corridor) = select_child_corridor(
                        current_corridor)

                # if you have found a high quality child, go for it
                else:
                    waiting_at_end_reached = False
                    (succes, current_corridor) = select_child_corridor(
                        current_corridor)

                # if we were planning on selecting a child but were unsuccesful,
                # wait a bit before backtracking
                if not succes:
                    # print("[manager] Need to backtrack!")
                    # if you are not waiting yet, start a timer
                    if not waiting_to_backtrack:
                        # print("[manager] Waiting to backtrack...")
                        backtrack_start_time = rospy.Time.now().to_sec()
                        waiting_to_backtrack = True

                    # if you are waiting to backtrack already, check if you can
                    # start backtracking
                    else:
                        backtrack_curr_time = rospy.Time.now().to_sec()
                        # print("[manager] Waiting to backtrack...")
                        if (backtrack_curr_time - backtrack_start_time >
                           backtrack_waiting_time):
                            # print("[manager] Started to backtrack")
                            waiting_to_backtrack = False
                            # pass the backtracking corridors to the motion
                            # planner
                            (backtrack_point, current_corridor,
                             backtracking_corridors, orphanage) = \
                                get_back_track_point(current_corridor,
                                                     orphanage,
                                                     explore_full_corridor)
                            if backtrack_point is None:
                                # print("[manager] ERROR: I cannot backtrack")
                                backtrack_mode_activated = False
                            else:
                                # print(f"[manager] Publishing {len(backtracking_corridors)} backtracking corridors")
                                # clear all backtrack-like corridors from the tree
                                clear_backtrack_like_corridors(root_corridor, orphanage)

                                publish_corridors(backtracking_corridors,
                                                  corridor_pub, backtrack_mode_activated)
                                backtrack_mode_activated = True

                
                # If we succesfully selected a child, publish it and proceed
                else:
                    # print("[manager] Selected child corridor")
                    waiting_to_backtrack = False
                    publish_corridors([current_corridor], corridor_pub,
                                      backtrack_mode_activated)

        if current_corridor is not None:
            visualize_corridor_tree(root_corridor, current_corridor, margin=.1)

        rate.sleep()


if __name__ == '__main__':
    main()
