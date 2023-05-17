#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Vector3, Pose2D
import math
import numpy as np

from corridor_world import CorridorWorld
from barn_challenge.msg import CorridorWorldMsg, CorridorWorldListMsg,\
    ManeuverMsg, GoalMsg

from motion_planner import planner, check_inside_one_point, compute_goal_point, planner_corridor_sequence, compute_initial_point


class messageClass():
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0


def yawFromQuaternion(orientation):
    return math.atan2((2.0*(orientation.w * orientation.z +
                            orientation.x * orientation.y)),
                      (1.0 - 2.0*(orientation.y * orientation.y +
                                  orientation.z * orientation.z)))


def odomCallback(data):
    message.posx = data.x
    message.posy = data.y
    message.theta = data.theta


def goalPositionCallback(data):
    global goal
    goal = data.goal


def corridorListCallback(data):
    global list_of_corridors
    global BACKTRACKING
    global EXECUTING_BACKTRACKING
    global GOAL_IN_SIGHT
    global RESUME_PLANNING

    if not GOAL_IN_SIGHT: # Avoid reading new corridors once the goal is in sight
        if data.len == 1:
            print("[navigator] .... got new corridor")
            RESUME_PLANNING = True
            for corridor_message in data.corridors:
                corridor_instance = CorridorWorld(
                    corridor_message.width_global,
                    corridor_message.height_global,
                    corridor_message.center_global,
                    corridor_message.tilt_global)
                corridor_instance.growth_center = corridor_message.growth_center_global
                if corridor_message.goal_in_sight:
                    list_of_corridors = [corridor_instance]
                    GOAL_IN_SIGHT = True
                else:
                    list_of_corridors.append(corridor_instance)
            BACKTRACKING = False
            EXECUTING_BACKTRACKING = False
        elif data.len > 1:
            print("[navigator] .... got new corridors for backtracking")
            # TODO: If more than one corridor is sent, backtrack
            list_of_corridors = []
            for corridor_message in data.corridors:
                corridor_instance = CorridorWorld(
                    corridor_message.width_global,
                    corridor_message.height_global,
                    corridor_message.center_global,
                    corridor_message.tilt_global)
                list_of_corridors.append(corridor_instance)
            # print(f"[navigator] Backtracking... got {data.len} corridors")
            BACKTRACKING = True



def generate_path_message(input_path):
    # TODO: check that it works correctly with the world frame
    path = Path()
    if input_path.shape[0] == 0:
        return path
    for i in range(input_path.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = input_path[i, 0]
        pose.pose.position.y = input_path[i, 1]
        pose.pose.position.z = 0
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
    return path


def generate_maneuver_message(maneuver_array):
    maneuver_msg = ManeuverMsg()
    maneuver_msg.len = maneuver_array.shape[0]
    x0_msg = Vector3()
    xf_msg = Vector3()
    x0_msg.x, x0_msg.y, x0_msg.z = message.posx, message.posy, message.theta
    if GOAL_IN_SIGHT:
        xf_msg.x, xf_msg.y, xf_msg.z = goal[0], goal[1], 0.
    else:
        xf_msg.x, xf_msg.y = compute_goal_point(list_of_corridors[0], 0.3)
        xf_msg.z = list_of_corridors[0].tilt
    maneuver_msg.x0.append(x0_msg)
    maneuver_msg.xf.append(xf_msg)
    for i in range(maneuver_array.shape[0]):
        part_maneuver = Vector3()
        part_maneuver.x = maneuver_array[i, 0]
        part_maneuver.y = maneuver_array[i, 1]
        part_maneuver.z = maneuver_array[i, 2]
        maneuver_msg.maneuver.append(part_maneuver)
    return maneuver_msg


def main():
    global message
    message = messageClass()
    global b_corridor
    b_corridor = CorridorWorldMsg()

    global BACKTRACKING
    BACKTRACKING = False

    global EXECUTING_BACKTRACKING
    EXECUTING_BACKTRACKING = False

    global list_of_corridors
    list_of_corridors = []

    global GOAL_IN_SIGHT
    GOAL_IN_SIGHT = False
    HEADING_TO_GOAL = False

    global RESUME_PLANNING
    RESUME_PLANNING = False

    # Subscribers
    odom_sub = rospy.Subscriber('/pose_map', Pose2D, odomCallback)
    corridor_list_sub = rospy.Subscriber('/chosen_corridor',
                                         CorridorWorldListMsg,
                                         corridorListCallback)

    global goal
    goal = np.array([0, 10])
    goal_sub = rospy.Subscriber('/goal_position', GoalMsg, goalPositionCallback)

    # Publishers
    path_Pub = rospy.Publisher('/path_corridors', Path, queue_size=1)
    maneuver_Pub = rospy.Publisher('/maneuver', ManeuverMsg, queue_size=1)

    rospy.init_node('navigator', anonymous=True)
    navigator_rate = 10
    rate = rospy.Rate(navigator_rate)

    # v_max = 0.5
    # v_min = -0.5
    # omega_max = 1.57
    # omega_min = -1.57
    v_max = 0.5
    v_min = -0.3
    omega_max = 0.3
    omega_min = -0.3
    u_bounds = np.array([v_min, v_max, omega_min, omega_max])
    Bck_vel_gain = 1
    u_bounds_back = np.array([Bck_vel_gain*v_min, Bck_vel_gain*v_max, Bck_vel_gain*omega_min, Bck_vel_gain*omega_max])
    a = 0.430
    b = 0.3
    m = 0.10

    print("[navigator] Initializing Navigator")

    STOP_PLANNING = False
    pos_restart_planning = None
    isDone = False
    message.goalx = message.posx
    message.goaly = message.posy + 10

    while not rospy.is_shutdown():

        # print('theta',message.theta-np.pi/2)
        #####################################################
        # Compute path and maneuver within corridors
        #####################################################

        if STOP_PLANNING:
            # check if pos_restart_planning is None -> store it
            if pos_restart_planning is None:
                pos_restart_planning = list_of_corridors[1].growth_center
                timer = rospy.Time.now().to_sec()
                RESUME_PLANNING = False

            print('nav growth center',list_of_corridors[1].growth_center)
            x_gc, y_gc = pos_restart_planning
            x_c, y_c = message.posx, message.posy
            dist = np.sqrt((x_gc - x_c)**2 + (y_gc - y_c)**2)
            print('dist', dist)
            if dist < 0.15 or rospy.Time.now().to_sec() - timer > 10:
                STOP_PLANNING = False
                pos_restart_planning = None
            rate.sleep()
            continue

        if len(list_of_corridors) > 0:
            print('tilt',list_of_corridors[0].tilt)
            if not BACKTRACKING:
                corridor1 = list_of_corridors[0]
                corridor2 = (list_of_corridors[1]
                            if len(list_of_corridors) > 1
                            else None)

                # TODO: implement/check the logic on removing first corridor from
                # list (.pop(0)) and shifting the corridors. First corridor should
                # always be the one where the robot is.

                if corridor2 is not None and check_inside_one_point(
                    # CorridorWorld(corridor2.width-(a), corridor2.height, corridor2.center, corridor2.tilt),
                    corridor2, 
                    np.array([message.posx, message.posy])):
                    ##############################
                    list_of_corridors.pop(0)
                    ## Get rid of first corridor as soon as you are already in the
                    ## second corridor
                    ##############################

                    ##############################
                    # goal_corridor_2 = compute_goal_point(corridor2, 0)
                    # goal_heading2 = np.arctan2(message.posx - goal_corridor_2[0], goal_corridor_2[1] - message.posy)

                    # c = np.sqrt((goal_corridor_2[1]-message.posy)**2 + (goal_corridor_2[0]-message.posx)**2) # distance to corridor goal
                    # threshold2 = np.abs(np.arctan((corridor2.width/2 - a/2 - m)/c))
                    # threshold2 = 0.25

                    # print(f"\tdiff heading: {np.abs((message.theta - np.pi/2) - corridor2.tilt) <= threshold2}")
                    # if np.abs((message.theta - np.pi/2) - corridor2.tilt) <= threshold2:
                    #     list_of_corridors.pop(0)
                    ##############################

                if not check_inside_one_point(corridor1, np.array([goal[0], goal[1]])):
                    # Compute the maneuvers within the corridors (by Sonia).
                    # Be aware that the tilt angle of the vehicle should be
                    # measured from the x-axis of the world frame
                    computed_maneuver, computed_path, poses, STOP_PLANNING = planner(
                        corridor1=corridor1,
                        u_bounds=u_bounds,
                        a=a, b=b, m=m,
                        x0=message.posx, y0=message.posy, theta0=message.theta,
                        plot=False,
                        corridor2=corridor2)
                    # The computed maneuver should be sent to the controller, which
                    # will define the instantaneous twist to be sent to the robot
                    maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
                    # Publish computed path for visualization on RViz
                    path_Pub.publish(generate_path_message(computed_path))

                else:
                    if GOAL_IN_SIGHT and not HEADING_TO_GOAL:
                        # Compute the maneuvers within the corridors (by Sonia).
                        # Be aware that the tilt angle of the vehicle should be
                        # measured from the x-axis of the world frame
                        computed_maneuver, computed_path, poses, STOP_PLANNING = planner(
                            corridor1=corridor1,
                            u_bounds=np.array([v_min, v_max, omega_min, omega_max]),
                            a=a, b=b, m=m,
                            x0=message.posx, y0=message.posy, theta0=message.theta,
                            plot=False,
                            corridor2=corridor2,
                            xf=goal[0], yf=goal[1])
                        computed_maneuver[-1][0] = 2.
                        print("[navigator] Heading to the goal", goal[0], goal[1])
                    
                    
                        # print(computed_maneuver)
                        # if not isDone:
                        # The computed maneuver should be sent to the controller, which
                        # will define the instantaneous twist to be sent to the robot
                        maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
                        # Publish computed path for visualization on RViz
                        path_Pub.publish(generate_path_message(computed_path))
                        # isDone = True
                        # GOAL_IN_SIGHT = True
                        HEADING_TO_GOAL = True
                    else:
                        #################################
                        c = np.sqrt((goal[1]-message.posy)**2 + (goal[0]-message.posx)**2) # distance to corridor goal
                        threshold = np.abs(np.arctan((corridor1.width/2 - a/2 - m)/c))
                        #################################
                        goal_heading = np.arctan2(message.posx - goal[0], goal[1] - message.posy)
                        print(f"[navigator] Difference in heading is {round(np.abs(message.theta - np.pi/2 - goal_heading)/2/np.pi*360, 3)}")
                        if np.abs(message.theta - np.pi/2 - goal_heading) >= threshold:
                            computed_maneuver, computed_path, poses, STOP_PLANNING = planner(
                                corridor1=corridor1,
                                u_bounds=np.array([v_min, v_max, omega_min, omega_max]),
                                a=a, b=b, m=m,
                                x0=message.posx, y0=message.posy, theta0=message.theta,
                                plot=False,
                                corridor2=corridor2,
                                xf=goal[0], yf=goal[1])
                            computed_maneuver[-1][0] = 2.
                            print("[navigator] Heading to the goal", goal[0], goal[1])

                            # print(computed_maneuver, "case 3")
                            maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
                            path_Pub.publish(generate_path_message(computed_path))

                        else:
                            print(f"[navigator] Skipping computing maneuver...already heading to goal {goal[0]},{goal[1]}")

            else: # Backtracking:
                # backtracking_list = list_of_corridors.reverse() # .reverse() does not return anything, it modifies the list

                ############################################################
                # Implementation using forward mode (and reversing maneuver)
                ############################################################
                # backtracking_list = list_of_corridors.copy()
                # backtracking_list.reverse()
                # print(f"[navigator] Received backtracking trigger with {len(backtracking_list)} corridors")
                # goal_point_last_corridor = compute_initial_point(backtracking_list[0], m)
                # fwd_computed_maneuver, computed_path, poses = planner_corridor_sequence(
                #     backtracking_list, 
                #     u_bounds_back, 
                #     a, 
                #     b, 
                #     m, 
                #     False, 
                #     goal_point_last_corridor[0], 
                #     goal_point_last_corridor[1], 
                #     backtracking_list[0].tilt+np.pi/2,
                #     xf = message.posx,
                #     yf = message.posy,
                #     # thetaf = message.theta0+np.pi/2
                # )
                # computed_maneuver = np.empty((0,fwd_computed_maneuver.shape[1]))
                # for maneuver_segment in fwd_computed_maneuver:
                #     computed_maneuver = np.vstack((np.array([-maneuver_segment[0], -maneuver_segment[1], maneuver_segment[2]]), computed_maneuver))

                ############################################################
                # Implementation using forward mode (and reversing maneuver)
                ############################################################
                if not EXECUTING_BACKTRACKING: # TODO check this
                    backtracking_list = []
                    for corridor_instance in list_of_corridors:
                    # for i in range(2):
                        # corridor_instance = list_of_corridors[i]
                        # Add corridor rotated 180 degrees
                        tilt = corridor_instance.tilt + np.pi
                        # print('orig tilt',corridor_instance.tilt)
                        # print('tilt',tilt, corridor_instance)
                        backtracking_list.append(CorridorWorld(corridor_instance.width, corridor_instance.height, corridor_instance.center, tilt))
                    
                    print(f"[navigator] Received backtracking trigger with {len(backtracking_list)} corridors")
                    goal_point_last_corridor = compute_initial_point(backtracking_list[-1], m)
                    # print(f"### Backtracking: veh angle = {message.theta-np.pi}, corridor angle = {backtracking_list[0].tilt}")
                    # print('theta for planner sequence', message.theta + np.pi - np.pi/2)
                    fwd_computed_maneuver, computed_path, poses = planner_corridor_sequence(
                        backtracking_list, 
                        u_bounds_back, 
                        a, 
                        b, 
                        m, 
                        False, 
                        x0 = message.posx, 
                        y0 = message.posy, 
                        theta0 = message.theta + np.pi,
                        xf = goal_point_last_corridor[0],
                        yf = goal_point_last_corridor[1],
                    )
                    print("[navigator] Forward-Backtracking maneuver computed")
                    print(fwd_computed_maneuver)
                    computed_maneuver = np.empty((0,fwd_computed_maneuver.shape[1]))
                    for maneuver_segment in fwd_computed_maneuver:
                        computed_maneuver = np.vstack((computed_maneuver, np.array([-maneuver_segment[0], maneuver_segment[1], maneuver_segment[2]])))


                    print("[navigator] Backtracking maneuver computed")
                    print(computed_maneuver)
                    # computed_maneuver = np.array([0,0,0]).reshape(1,3)
                    # # The computed maneuver should be sent to the controller, which
                    # # will define the instantaneous twist to be sent to the robot
                    maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
                    # Publish computed path for visualization on RViz
                    path_Pub.publish(generate_path_message(computed_path))

                    EXECUTING_BACKTRACKING = True

        # Finish execution if goal has been reached
        if isDone:
            rospy.signal_shutdown('Goal Reached')

        rate.sleep()


if __name__ == '__main__':
    main()
