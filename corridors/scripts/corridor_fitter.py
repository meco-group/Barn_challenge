#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor
from barn_challenge.msg import CorridorLocalMsg
from corridor_helpers import *


class messageClass():
    def __init__(self):
        self.velx = 0.0
        self.rotz = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.scandata = None
        self.ranges = []
        self.angle_inc = None
        self.angles = []
        self.minind = 0
        self.maxind = 719
        self.angle_min = None
        self.pointcloud = None
        self.scanCoords = []
        self.corridors = []


def scanCallback(data):
    ind_min = message.minind
    ind_max = message.maxind
    message.scandata = np.asarray(data.ranges[ind_min:ind_max])
    message.angle_inc = data.angle_increment
    if not len(message.angles):
        message.angles = np.array([data.angle_min + i*data.angle_increment
                                   for i in range(ind_min, ind_max)])
        message.angle_min = data.angle_min


def pointCallback(data):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lg.LaserProjection().projectLaser(data)
    message.pointcloud = pc2_msg


def odomCallback(data):
    message.velx = data.twist.twist.linear.x
    message.rotz = data.twist.twist.angular.z

    message.posx = data.pose.pose.position.x
    message.posy = data.pose.pose.position.y
    message.theta = yaw_from_quaternion(data.pose.pose.orientation)


def yaw_from_quaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))


def wrap_to_pi(angle):
    if angle > m.pi:
        angle -= 2*m.pi
    elif angle < -m.pi:
        angle += 2*m.pi
    return angle


def main():
    global message
    message = messageClass()

    global fitter_rate
    fitter_rate = .5
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    scan_sub = rospy.Subscriber('/front/scan', LaserScan, scanCallback)
    point_sub = rospy.Subscriber("/front/scan", LaserScan, pointCallback,
                                 queue_size=1)
    corridor_pub = rospy.Publisher("/corridor", CorridorLocalMsg, queue_size=10)

    rospy.init_node('fitter', anonymous=True)
    rate = rospy.Rate(fitter_rate)

    print('Building initial corridor')
    corridor_0 = Corridor(width=2., height=2., center=[0, 0],
                          tilt=0, copy=True)
    message.corridors.append(corridor_0)
    corridor_0.get_vertices_for_visualization(
        message.posx, message.posy, message.theta)

    best_corridors = [corridor_0]
    last_best_corr = best_corridors[-1]
    active = 1

    print('Corridor node started')
    while not rospy.is_shutdown() and active:
        # Convert laser scan data to point cloud and homogenous coordinates.
        for xy in last_best_corr.corners_world:
            if xy.x > 8:
                active = 0
                print('Finishing and building last corridor')

        point_generator = pc2.read_points(message.pointcloud)
        xyh = []
        i = 0
        for point in point_generator:
            i = i + 1
            if i % 4 == 0:  # Only select 1/4th of points.
                xyh.append([point[0], point[1], 1])
        xy = np.array([value for value in xyh]).T

        print('Building new corridors')
        x, y, theta = message.posx, message.posy, message.theta
        last_best_corr = best_corridors[-1]
        centers = [[0, 0]]
        centers = centers + centers + centers
        tilts = [0, np.pi/6, -np.pi/6]
        last_best_corr.sweep_corridor(width=.6, height=1.,
                                      centers=centers, tilts=tilts, copy=True)

        for i, corridor in enumerate(last_best_corr.children):
            corridor.grow_all_edges(xy)
            message.corridors.append(corridor)  # TODO do dit weg
            corridor.get_vertices_for_visualization(x, y, theta)
            corridor.rviz_visualization('rect_fitter', i, 0., 0., 0.7,
                                        1/fitter_rate)

        next_best_corr = sorted(last_best_corr.children, key=lambda corridor:
                                max([xs.y for xs in corridor.corners_world]),
                                reverse=True)[:1]

        best_corridors += [next_best_corr[0]]

        for i, corridor in enumerate(best_corridors):
            corridor.rviz_visualization('rect_fitter',
                                        i+100, 0.7, 0.7, 0.7, 1/fitter_rate)

        # Create corridor message for communication
        corridor_msg = CorridorLocalMsg()
        corridor_msg.height_local = next_best_corr[0].height
        corridor_msg.width_local = next_best_corr[0].width
        corridor_msg.quality_local = next_best_corr[0].quality
        corridor_msg.center_local = next_best_corr[0].center
        corridor_msg.growth_center_local = next_best_corr[0].growth_center
        corridor_msg.tilt_local = next_best_corr[0].tilt
        corridor_msg.init_pos_global = [x, y, theta]

        xy_corners = []
        for xy in next_best_corr[0].corners_world:
            xy_corners.append(xy.x)
            xy_corners.append(xy.y)
        corridor_msg.corners_local = xy_corners
        corridor_pub.publish(corridor_msg)

        rate.sleep()


if __name__ == '__main__':
    main()
