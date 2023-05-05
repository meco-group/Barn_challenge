#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Vector3
import math
import numpy as np

from corridor_world import CorridorWorld
from barn_challenge.msg import CorridorWorldMsg, CorridorWorldListMsg,\
    ManeuverMsg, AngleListMsg

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
    message.velx = data.twist.twist.linear.x
    message.rotz = data.twist.twist.angular.z
    message.posx = data.pose.pose.position.x
    message.posy = data.pose.pose.position.y
    message.theta = yawFromQuaternion(data.pose.pose.orientation)

def goalPositionCallback(data):
    global goal
    goal = data.angles.copy()
    print("Goal position callback executed!")

def corridorListCallback(data):
    global list_of_corridors
    global BACKTRACKING
    global GOAL_IN_SIGHT
    if not GOAL_IN_SIGHT:
        print(".... got new corridor(s)")
        if data.len == 1:
            for corridor_message in data.corridors:
                corridor_instance = CorridorWorld(
                    corridor_message.width_global,
                    corridor_message.height_global,
                    corridor_message.center_global,
                    corridor_message.tilt_global)
                list_of_corridors.append(corridor_instance)
            BACKTRACKING = False
        elif data.len > 1:
            # TODO: If more than one corridor is sent, backtrack
            print("[navigator] Backtracking...")
            list_of_corridors = []
            for corridor_message in data.corridors:
                corridor_instance = CorridorWorld(
                    corridor_message.width_global,
                    corridor_message.height_global,
                    corridor_message.center_global,
                    corridor_message.tilt_global)
                list_of_corridors.append(corridor_instance)
            BACKTRACKING = True



def generate_path_message(input_path):
    # TODO: check that it works correctly with the world frame
    path = Path()
    if input_path.shape[0] == 0:
        return path
    for i in range(input_path.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = input_path[i, 0]
        pose.pose.position.y = input_path[i, 1]
        pose.pose.position.z = 0
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = "odom"
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
        xf_msg.x, xf_msg.y, xf_msg.z = message.goalx, message.goaly, 0.
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

    global list_of_corridors
    list_of_corridors = []

    global GOAL_IN_SIGHT
    GOAL_IN_SIGHT = False

    # Subscribers
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    corridor_list_sub = rospy.Subscriber('/chosen_corridor',
                                         CorridorWorldListMsg,
                                         corridorListCallback)

    global goal
    goal = np.array([0, 10])
    goal_sub = rospy.Subscriber('/goal_position', AngleListMsg, goalPositionCallback)

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
    v_max = 0.3
    v_min = -0.3
    omega_max = 0.3
    omega_min = -0.3
    u_bounds = np.array([v_min, v_max, omega_min, omega_max])
    a = 0.430
    b = 0.508
    m = 0.07

    print("Initializing Navigator")

    isDone = False
    message.goalx = message.posx
    message.goaly = message.posy + 10

    while not rospy.is_shutdown():

        #####################################################
        # Compute path and maneuver within corridors
        #####################################################

        if len(list_of_corridors) > 0:

            if not BACKTRACKING:
                corridor1 = list_of_corridors[0]
                corridor2 = (list_of_corridors[1]
                            if len(list_of_corridors) > 1
                            else None)

                # TODO: implement/check the logic on removing first corridor from
                # list (.pop(0)) and shifting the corridors. First corridor should
                # always be the one where the robot is.

                if corridor2 is not None and check_inside_one_point(
                corridor2, np.array([message.posx, message.posy])):
                    # corridor1, corridor2 = corridor2, None
                    list_of_corridors.pop(0)
                    # Get rid of first corridor as soon as you are already in the
                    # second corridor

                if not check_inside_one_point(
                corridor1, np.array([message.goalx, message.goaly])):
                    # Compute the maneuvers within the corridors (by Sonia).
                    # Be aware that the tilt angle of the vehicle should be
                    # measured from the x-axis of the world frame
                    computed_maneuver, computed_path, poses = planner(
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
                    
                    # Compute the maneuvers within the corridors (by Sonia).
                    # Be aware that the tilt angle of the vehicle should be
                    # measured from the x-axis of the world frame
                    computed_maneuver, computed_path, poses = planner(
                        corridor1=corridor1,
                        u_bounds=u_bounds,
                        a=a, b=b, m=m,
                        x0=message.posx, y0=message.posy, theta0=message.theta,
                        plot=False,
                        corridor2=corridor2,
                        xf=message.goalx, yf=message.goaly)
                    print("[navigator] Heading to the goal")
                    
                    if not GOAL_IN_SIGHT:
                        # print(computed_maneuver)
                        # if not isDone:
                        # The computed maneuver should be sent to the controller, which
                        # will define the instantaneous twist to be sent to the robot
                        maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
                        # Publish computed path for visualization on RViz
                        path_Pub.publish(generate_path_message(computed_path))
                        # isDone = True
                    GOAL_IN_SIGHT = True

            else: # Backtracking:
                backtracking_list = list_of_corridors.reverse()
                goal_point_last_corridor = compute_initial_point(backtracking_list[0], m)
                fwd_computed_maneuver, computed_path, poses = planner_corridor_sequence(
                    backtracking_list, 
                    u_bounds, 
                    a, 
                    b, 
                    m, 
                    False, 
                    goal_point_last_corridor[0], 
                    goal_point_last_corridor[1], 
                    backtracking_list[0].tilt
                )
                computed_maneuver = np.empty((0,fwd_computed_maneuver.shape[1]))
                for maneuver_segment in fwd_computed_maneuver:
                    computed_maneuver = np.vstack((np.array([-maneuver_segment[0], -maneuver_segment[1], maneuver_segment[2]]), computed_maneuver))

                print("[navigator] Backtracking maneuver computed")
                print(computed_maneuver)
                # The computed maneuver should be sent to the controller, which
                # will define the instantaneous twist to be sent to the robot
                maneuver_Pub.publish(generate_maneuver_message(computed_maneuver))
                # Publish computed path for visualization on RViz
                path_Pub.publish(generate_path_message(computed_path))

        # Finish execution if goal has been reached
        if isDone:
            rospy.signal_shutdown('Goal Reached')

        rate.sleep()


if __name__ == '__main__':
    main()
