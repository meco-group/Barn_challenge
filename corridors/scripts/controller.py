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
from barn_challenge.msg import ManeuverMsg


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


def ManeuverCallback(data):
    global v_full, w_full

    length = data.len
    maneuver = data.maneuver
    vx, wz, t, v_full, w_full = [], [], [], [], []
    for i in range(length):
        vx.append(maneuver[i].x)
        wz.append(maneuver[i].y)
        t.append(maneuver[i].z)

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
    vmax = .3
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

    # Subscribers
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
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
    message.goalx = message.posx
    message.goaly = message.posy + 10

    global v_full, w_full
    v_full = []
    w_full = []

    print('I started moving')
    while not rospy.is_shutdown():

        if len(v_full) > 0:
            twist.linear.x = v_full.pop(0)
            twist.angular.z = w_full.pop(0)
        else:
            twist.linear.x = 0.
            twist.angular.z = 0.

        # print('current_position', message.posx, message.posy, message.theta)
        # print('current_velocity', twist.linear.x, twist.angular.z)

        vel_Pub.publish(twist)
        # This should go in a controller node (replace with maneuver publisher)

        distToGoal = distance((message.goalx, message.goaly),
                              (message.posx, message.posy))

        if distToGoal < 0.5:
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
