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


class messageClass():
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0


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
    corridor_sub = rospy.Subscriber('/corridor', corridor_msg, corridorCallback)

    rospy.init_node('manager', anonymous=True)
    rate = rospy.Rate(1)

    print('manager ready')
    while not rospy.is_shutdown():
        print('manager managing management stuff')
        print(b_corridor.height)

        rate.sleep()


if __name__ == '__main__':
    main()
