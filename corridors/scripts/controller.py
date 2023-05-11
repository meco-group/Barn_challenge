#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from geometry_msgs.msg import Twist, Pose2D
import math as m
import numpy as np
from barn_challenge.msg import ManeuverMsg, GoalMsg


class messageClass():
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.man_vx = []
        self.man_wz = []
        self.man_t = []


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


def odomCallback(data):
    message.posx = data.x
    message.posy = data.y
    message.theta = data.theta


def goalPositionCallback(data):
    global goal
    goal = data.goal


def ManeuverCallback(data):
    global v_full, w_full

    length = data.len
    maneuver = data.maneuver
    vx, wz, t, v_full, w_full = [], [], [], [], []
    for i in range(length):
        vx.append(maneuver[i].x)
        wz.append(maneuver[i].y)
        t.append(maneuver[i].z)

    # convert short period of vmax by longer period of lower
    # velocity to combat slipping
    time_burst_threshold = 1.0
    vmax = 0.5
    if data.len >= 2 and vx[0] >= 0.4 and vx[1] <= 0.1 and \
            t[0] < time_burst_threshold:
        vx[0] = max(0.1, vmax*t[0]/time_burst_threshold)
        t[0] = time_burst_threshold
    elif data.len == 1 and vx[0] >= 0.4 and \
            t[0] < time_burst_threshold:
        vx[0] = max(0.1, vmax*t[0]/time_burst_threshold)
        t[0] = time_burst_threshold

    for time, v, w in zip(t, vx, wz):
        if not m.isnan(time):
            repetitions = int(time*controller_rate)
            v_full += [v]*repetitions
            w_full += [w]*repetitions

    # print(data.x0)
    x0, y0, theta0 = data.x0[0].x, data.x0[0].y, data.x0[0].z
    xf, yf, thetaf = data.xf[0].x, data.xf[0].y, data.xf[0].z

    dist = distance((x0, y0), (xf, yf))
    # print('theta0', theta0)
    # print('thetaf', thetaf)
    # print('dist', dist)
    # print('arctan', np.arctan2(yf - y0, xf - x0) - theta0)
    # print(dist, np.arctan2(yf - y0, xf - x0) - thetaf)
    # print(dist, np.arctan2(yf - y0, xf - x0) - thetaf)

    repetitions = 15
    vmax = .5
    wmax = .5
    # # first minimize heading error
    # if thetaf - theta0 + np.pi/2 > np.pi/8:
    #     w_full = [vmax]*repetitions
    #     v_full = [0.]*repetitions
    #     print('pure rotation CC')
    # elif thetaf - theta0 + np.pi/2 < -np.pi/8:
    #     w_full = [-vmax]*repetitions
    #     v_full = [0.]*repetitions
    #     print('pure rotation C')
    # else:
    #     v_full = [vmax]*repetitions
    #     print(np.arctan2(yf - y0, xf - x0) - theta0)
    #     if np.arctan2(yf - y0, xf - x0) - theta0 > 0.05:
    #         w_full = [wmax]*repetitions
    #         print('translation + rotation CC')
    #     elif np.arctan2(yf - y0, xf - x0) - theta0 < -0.05:
    #         w_full = [-wmax]*repetitions
    #         print('translation + rotation C')
    #     else:
    #         w_full = [0.]*repetitions
    #         print('pure translation')
    # print('dist', dist)
    
    
    # if dist < 1.:
    #     print(dist*vmax)
    #     v_full = [dist*vmax]*repetitions
    #     w_full = [0.]*repetitions


def main():

    global controller_rate
    controller_rate = 100

    global message
    message = messageClass()

    global maneuver_mes

    global goal
    goal = np.array([0, 10])
    goal_sub = rospy.Subscriber('/goal_position', GoalMsg, goalPositionCallback)

    # Subscribers
    odom_sub = rospy.Subscriber('/pose_map', Pose2D, odomCallback)
    maneuver_sub = rospy.Subscriber('/maneuver', ManeuverMsg, ManeuverCallback)

    # Publishers
    vel_Pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                              queue_size=100)

    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(controller_rate)

    isDone = False
    twist = Twist()
    twist.linear.x = 0.
    twist.angular.z = 0.
    message.goalx = goal[0]
    message.goaly = goal[1]

    global v_full, w_full
    v_full = []
    w_full = []
    v_previous = 0
    w_previous = 0

    smoother = .05

    print('I started moving')
    while not rospy.is_shutdown():

        if len(v_full) > 0:
            v_next = v_full.pop(0)
            w_next = w_full.pop(0)
            twist.linear.x = v_next*smoother + v_previous*(1-smoother)
            twist.angular.z = w_next*smoother + w_previous*(1-smoother)
            v_previous = twist.linear.x
            w_previous = twist.angular.z
        else:
            twist.linear.x = 0.
            twist.angular.z = 0.

        # print('current_position', message.posx, message.posy, message.theta)
        # print('current_velocity', twist.linear.x, twist.angular.z)

        vel_Pub.publish(twist)
        # This should go in a controller node (replace with maneuver publisher)

        distToGoal = distance((goal[0], goal[1]),
                              (message.posx, message.posy))

        if distToGoal < 0.1:
            twist.linear.x = 0.
            twist.angular.z = 0.
            print('I arrived')
            isDone = True

        # Finish execution if goal has been reached
        if isDone:
            rospy.signal_shutdown('Goal Reached')

        rate.sleep()


if __name__ == '__main__':
    main()
