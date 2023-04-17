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

class lidarData():
    def __init__(self):
        self.sensor_num = np.int(720)
        self.lidar_data = np.zeros(self.sensor_num)

def scanCallback(data):
    lidar_data.lidar_data = np.asarray(data.ranges) #need to confirm if this syntax is correct
    lidar_data.lidar_data[lidar_data.lidar_data == float('inf')] = 10.
    #print(lidar_data.lidar_data)

def main():
    global lidar_data
    lidar_data = lidarData()
    sensor_span = (3/2)*(np.pi)
    lidar_resolution = sensor_span/lidar_data.sensor_num #angle resolution in radians
    sector_num = 5 # number of sectors
    sector_size = np.int(lidar_data.sensor_num/sector_num) # number of points per sector
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
        prev_sec = 0.0
        start_angle = 0.0
        end_angle = 0.0
        free_angles = np.array([]) # array of free angles to build corridors in those directions
        opening_found = False
        for i in range(sector_num): # loop through all sectors
            x = lidar_data.lidar_data[i*sector_size:(i+1)*sector_size]
            x_ordered = np.argsort(x)
            for j in range(sector_size): # loop through data on a sector
                x_index = x_ordered[j]
                arc_length = lidar_resolution*x[x_index]
                opening_width = arc_length/2
                opening_found = False
                for k in range(sector_size):
                    if x[k] > x[x_index]: #TODO: may need an extra condition if values are 'inf'
                        opening_width = opening_width + arc_length
                        if opening_width > (2*(robot_radius+safety_radius)): # compare if space is enough for a corridor
                            opening_found = True                            
                            break
                    else:
                        opening_width = opening_width + arc_length/2
                        if opening_width > (2*(robot_radius+safety_radius)): # compare if space is enough for a corridor
                            opening_found = True
                            break
                        opening_width = 0.0
                if opening_found == True:
                    if prev_sec == 0.0: # if it is a new free sector, define start and end angles
                        start_angle = i*lidar_resolution*sector_size
                        end_angle = (i+1)*lidar_resolution*sector_size
                    else: # if it is a consecutive sector, only update the end angle
                        end_angle = (i+1)*lidar_resolution*sector_size
                    prev_sec = 1.0
                    break
            if opening_found == False: 
                sectors[i] = 0.0
                if prev_sec == 1.0: # if the current sector is occupied, but follows free sectors
                    #free_angles.append((start_angle + end_angle)/2) 
                    free_angles = np.append(free_angles, (start_angle + end_angle)/2) # add the middle angle of the free sectors
                prev_sec = 0.0
        if prev_sec == 1: # if the last sector is free, also send the middle angle of the last consecutive sectors
            #free_angles.append((start_angle + end_angle)/2)
            free_angles = np.append(free_angles, (start_angle + end_angle)/2)

        free_angles = free_angles - sensor_span/2 # change angles to range [-135,135] degrees (in radians)
        
        free_sectors = len(free_angles)
        if (free_sectors != last_free_sectors):
            flag = 1.0
        else:
            flag = 0.0
        last_free_sectors = free_sectors
        print(start_angle)
        print(end_angle)
        print('sectors',sectors)
        print('angles',free_angles)
        print('flag', flag)
        #publish free_angles
        #publish flag

        rate.sleep()


if __name__ == '__main__':
    main()
