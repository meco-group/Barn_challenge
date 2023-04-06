#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor
from barn_challenge.msg import corridor_msg
from barn_challenge.msg import corridor_list

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


def corridorCallback(data):
    b_corridor.height = data.height
    b_corridor.width = data.width
    b_corridor.quality = data.quality
    b_corridor.center = data.center
    b_corridor.tilt = data.tilt
    b_corridor.corners = data.corners


def main():
    global message
    message = messageClass()
    global b_corridor
    b_corridor = corridor_msg()
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback)

    vel_Pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                              queue_size=10)

    rospy.init_node('move_base_node', anonymous=True)
    rate = rospy.Rate(100)

    maxSpeed = 1
    minSpeed = 0.1
    maxTurn = m.pi/2
    isDone = False
    twist = Twist()
    twist.linear.x = 0.
    twist.angular.z = 0.
    message.goalx = message.posx
    message.goaly = message.posy + 10

    print('I started moving')
    while not rospy.is_shutdown():
        # Start moving slowly.
        twist.linear.x = 0.1
        twist.angular.z = 0.

        distToGoal = distance((message.goalx, message.goaly),
                              (message.posx, message.posy))

        if distToGoal < 0.5:
            twist.linear.x = 0.
            twist.angular.z = 0.
            print('I arrived')
            isDone = True

        print('test', b_corridor.height)
        vel_Pub.publish(twist)

        if isDone:
            rospy.signal_shutdown('Goal Reached')

        rate.sleep()


if __name__ == '__main__':
    main()
