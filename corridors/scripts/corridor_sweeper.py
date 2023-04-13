#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue April 11, 2023

@author: alex
"""

import rospy
from sensor_msgs.msg import LaserScan
import math as m
import numpy as np
from barn_challenge.msg import corridor_msg

def scanCallback(data):
    lidar_data = data.ranges #need to confirm if this syntax is correct

def main():
    global lidar_data
    lidar_data = []
    sensor_num = np.int(720)
    sensor_span = (3/2)*(np.pi)
    lidar_resolution = sensor_span/sensor_num #angle resolution in radians
    sector_num = 15 # number of sectors
    sector_size = np.int(sensor_num/sector_num) # number of points per sector
    free_sectors = 0.0
    last_free_sectors = 0.0
    flag = 0.0

    robot_radius = 0.5
    safety_radius = 0.1
    scan_sub = rospy.Subscriber('/front/scan', LaserScan, scanCallback)

    rospy.init_node('sweeper', anonymous=True)
    rate = rospy.Rate(10)

    print('sweeper ready')
    while not rospy.is_shutdown():
        
        sectors = np.full((sector_num), 1.0)
        free_angles = []
        for i in range(sector_num): # loop through sectors
            x = lidar_data[i*sector_size:(i+1)*sector_size]
            x_ordered = np.argsort(x)
            for j in range(sector_size): # loop through 
                x_index = x_ordered[j]
                arc_length = lidar_resolution*x[x_index]
                opening_width = arc_length/2
                opening_found = False
                for k in range(sector_size):
                    if x[k] > x[x_index]: #may need an extra condition if values are 'inf'
                        opening_width = opening_width + arc_length
                        if opening_width > (2*(robot_radius+safety_radius)):
                            opening_found = True
                            free_angles.append(i*lidar_resolution + lidar_resolution/2)
                            break
                    else:
                        opening_width = opening_width + arc_length/2
                        if opening_width > (robot_radius+safety_radius):
                            opening_found = True
                            free_angles.append(i*lidar_resolution + lidar_resolution/2)
                            break
                        opening_width = 0.0
                if opening_found == False:
                    sectors[i] = 0.0

        free_sectors = len(free_angles)
        if (free_sectors != last_free_sectors):
            flag = 1.0
        else:
            flag = 0.0
        last_free_sectors = free_sectors

        #publish free_angles
        #publish flag

        rate.sleep()


if __name__ == '__main__':
    main()
