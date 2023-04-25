#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue April 11, 2023

@author: alex
"""

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math as m
import numpy as np
from barn_challenge.msg import AngleListMsg


class lidarData():
    def __init__(self):
        self.sensor_num = int(720/2)  # Consider only half of the lidar range.
        self.lidar_data = np.zeros(self.sensor_num)


class odomData():
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.posz = 0.0
        self.yaw = 0.0


def yawFromQuaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))


def rotation_matrix(angle):
    J = np.array([[m.cos(angle), -m.sin(angle)],
                  [m.sin(angle), m.cos(angle)]])
    return (J)


def local_to_world(x2, y2, angle):
    p = np.array([x2, y2])
    J = rotation_matrix(angle)
    n = J.dot(p)
    world_x2 = n[0] + odom_data.posx
    world_y2 = n[1] + odom_data.posy
    return (world_x2, world_y2)


def wrapToPi(angle):
    if angle > m.pi:
        angle -= 2*m.pi
    elif angle < -m.pi:
        angle += 2*m.pi
    return angle


def scanCallback(data):
    lidar_data.lidar_data = np.asarray(data.ranges[(0+180):(719-179)])
    lidar_data.lidar_data[lidar_data.lidar_data == float('inf')] = 10.


def odomCallback(data):
    odom_data.posx = data.pose.pose.position.x
    odom_data.posy = data.pose.pose.position.y
    odom_data.posz = data.pose.pose.position.z
    odom_data.yaw = yawFromQuaternion(data.pose.pose.orientation)


def visualizeArrows(angles):
    marker_array = MarkerArray()
    for i in range(len(angles)):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'points_arrows'
        marker.lifetime = rospy.Time(.1)
        marker.id = i
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.scale.x = 0.02
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.w = 1.0
        tail = Point(odom_data.posx, odom_data.posy, odom_data.posz)
        free_angle = angles[i]
        angle = free_angle + odom_data.yaw
        angle = wrapToPi(angle)
        x2, y2 = local_to_world(2, 0, angle)
        tip = Point(x2, y2, odom_data.posz)
        marker.points = [tail, tip]
        marker_array.markers.append(marker)
    return marker_array


def main():
    global lidar_data
    global odom_data
    lidar_data = lidarData()
    odom_data = odomData()
    sensor_range_used = 1/2
    sensor_span = (3/2)*(np.pi)*sensor_range_used
    sensor_num = lidar_data.sensor_num
    lidar_resolution = sensor_span/sensor_num  # angle resolution
    sector_num = 6  # number of sectors
    sector_size = int(sensor_num/sector_num)  # number of points per sector
    free_sectors = 0.0
    last_free_sectors = 0.0
    flag = 0.0

    robot_radius = 0.25
    safety_radius = 0.1
    scan_sub = rospy.Subscriber('/front/scan', LaserScan, scanCallback)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    marker_pub = rospy.Publisher('/angles_marker', MarkerArray, queue_size=10)
    angle_pub = rospy.Publisher('/angle_list', AngleListMsg, queue_size=10)

    rospy.init_node('sweeper', anonymous=True)
    sweeper_rate = 5
    rate = rospy.Rate(sweeper_rate)

    print('sweeper ready')
    while not rospy.is_shutdown():
        sectors = np.full((sector_num), 1.0)
        prev_sec = 0.0
        start_angle = 0.0
        end_angle = 0.0

        # array of free angles to build corridors in those directions
        free_angles = np.array([])
        free_angles_all = np.array([])
        minimum_distance_array = np.array([])
        opening_found = False

        for i in range(sector_num):  # loop through all sectors
            x = lidar_data.lidar_data[i*sector_size:(i+1)*sector_size]
            x_ordered = np.argsort(x)
            for j in range(sector_size):  # loop through data on a sector
                x_index = x_ordered[j]
                arc_length = lidar_resolution*x[x_index]
                opening_width = arc_length/2
                opening_found = False
                if x[x_index] >= 0.1:
                    for k in range(sector_size):
                        if x[k] >= x[x_index]:
                            opening_width = opening_width + arc_length
                            # compare if space is enough for a corridor
                            if opening_width > 2*(robot_radius+safety_radius):
                                opening_found = True
                                start = i*lidar_resolution*sector_size + \
                                    lidar_resolution*k
                                end = i*lidar_resolution*sector_size + \
                                    lidar_resolution*x_index
                                free_angles_all = np.append(free_angles_all,
                                                            (start+end)/2)
                                minimum_distance_array = np.append(
                                    minimum_distance_array, x[x_index])
                                break
                        else:
                            opening_width = opening_width + arc_length/2
                            # compare if space is enough for a corridor
                            if opening_width > 2*robot_radius+safety_radius:
                                opening_found = True
                                start = i*lidar_resolution*sector_size + \
                                    lidar_resolution*k
                                end = i*lidar_resolution*sector_size + \
                                    lidar_resolution*x_index
                                free_angles_all = np.append(free_angles_all,
                                                            (start+end)/2)
                                minimum_distance_array = np.append(
                                    minimum_distance_array, x[x_index])
                                break
                            opening_width = 0.0

                    if opening_found:
                        # if new free sector, define start and end angles
                        if prev_sec == 0.0:
                            start_angle = i*lidar_resolution*sector_size
                            end_angle = (i+1)*lidar_resolution*sector_size
                        # if consecutive sector, only update the end angle
                        else:
                            end_angle = (i+1)*lidar_resolution*sector_size
                        prev_sec = 1.0
                        break

            if not opening_found:
                sectors[i] = 0.0
                # if current sector is occupied, but follows free sectors
                if prev_sec == 1.0:
                    # add the middle angle of the free sectors
                    free_angles = np.append(free_angles,
                                            (start_angle+end_angle)/2)
                prev_sec = 0.0

        # if the last sector is free, also send the middle angle of the last
        # consecutive sectors
        if prev_sec == 1:
            free_angles = np.append(free_angles, (start_angle+end_angle)/2)

        # change angles to range [-135,135] degrees (in radians)
        free_angles = free_angles - sensor_span*sensor_range_used
        free_angles_all = free_angles_all - sensor_span*sensor_range_used

        free_sectors = len(free_angles)
        if (free_sectors != last_free_sectors):
            flag = 1.0
        else:
            flag = 0.0

        last_free_sectors = free_sectors

        # print('sectors', sectors)
        # print('angles', free_angles_all)
        # print('min distance', minimum_distance_array)
        # print('flag', flag)

        angle_list_msg = AngleListMsg()
        angle_list_msg.angles = free_angles_all
        angle_pub.publish(angle_list_msg)

        marker_array = visualizeArrows(free_angles_all)
        marker_pub.publish(marker_array)

        rate.sleep()


if __name__ == '__main__':
    main()
