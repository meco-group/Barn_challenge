#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry, Path
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist, PoseStamped
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor

from motion_planner import compute_trajectory


class messageClass():
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0


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


def main():
    global message
    message = messageClass()

    # Subscribers
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)

    # Publishers
    vel_Pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                              queue_size=10)

    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(100)

    isDone = False
    twist = Twist()
    twist.linear.x = 0.
    twist.angular.z = 0.
    message.goalx = message.posx
    message.goaly = message.posy + 10

    v_compact = [.5, .5, .5, .5]
    w_compact = [1.57, 0., 1.57, 0.]
    t_compact = [0.51383007, 8.24802881, 0.38729874, 10.19587506]

    v_full = []
    w_full = []

    for time, v, w in zip(t_compact, v_compact, w_compact):
        while time > 0:
            time = time - .01
            v_full.append(v)
            w_full.append(w)

    print('I started moving')
    while not rospy.is_shutdown():
        if len(v_full) > 0:
            twist.linear.x = v_full.pop(0)
            twist.angular.z = w_full.pop(0)
        else:
            twist.linear.x = 0.
            twist.angular.z = 0.

        print('current_position', message.posx, message.posy, message.theta)
        print('current_velocity', twist.linear.x, twist.angular.z)

        vel_Pub.publish(twist)  # This should go in a controller node (replace with maneuver publisher)

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
