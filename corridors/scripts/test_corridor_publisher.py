#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan and louis
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math as m
import numpy as np
from barn_challenge.msg import corridor_msg, corridor_list
from corridor_helpers import *
from corridor import Corridor

def publishCorridors(corridor, publisher):
    to_send = corridor_msg()
    to_send.height = corridor.height
    to_send.width = corridor.width
    to_send.quality = corridor.quality
    to_send.center = corridor.center
    to_send.growth_center = corridor.growth_center
    to_send.tilt = corridor.tilt
    xy_corners = []
    for xy in corridor.corners_world:
        xy_corners.append(xy.x)
        xy_corners.append(xy.y)
    to_send.corners = xy_corners

    publisher.publish(to_send)
    print("[tester] Published a new corridor")

def main():

    c = Corridor(center=[0.0,0.0], width=0.5, height=2., tilt=np.pi/2)
    c.add_child_corridor(Corridor(center=[0.0,1.0], width=0.8, height=5.5, tilt=np.pi/2+np.pi/6))
    c.add_child_corridor(Corridor(center=[0.0,1.0], width=0.9, height=3.5, tilt=np.pi/2-np.pi/4))

    corridor_list = [c.children[0]]

    # Prepare to publish corridors
    corridor_pub = rospy.Publisher("/corridor", corridor_msg)

    rospy.init_node('tester', anonymous=True)
    rate = rospy.Rate(3)

    # Define variables for the corridor tree
    done = False
    counter = 0

    print('[tester] tester is ready')
    while not rospy.is_shutdown():
        # print('[tester] tester looping')
        if not done and counter == 8:
            # publishCorridors(corridor_list[0], corridor_pub)
            publishCorridors(Corridor(center=[0.0,0.0], width=0.5, height=2., tilt=0), corridor_pub)
        if not done and counter == 9:
            publishCorridors(Corridor(center=[1.0,0.0], width=0.8, height=3.5, tilt=np.pi/6), corridor_pub)
        if not done and counter == 10:
            publishCorridors(Corridor(center=[1.0,0.0], width=0.9, height=4.5, tilt=-np.pi/4), corridor_pub)
            done = True

        counter += 1
        rate.sleep()


if __name__ == '__main__':
    main()
