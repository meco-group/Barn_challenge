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
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor
from corridors.msg import corridor_msg
from corridors.msg import corridor_list

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


def yawFromQuaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))


def scanCallback(data):
    ind_min = message.minind
    ind_max = message.maxind
    message.scandata = np.asarray(data.ranges[ind_min:ind_max])
    message.angle_inc = data.angle_increment
    if not len(message.angles):
        message.angles = np.array([data.angle_min + i * data.angle_increment
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
    message.theta = yawFromQuaternion(data.pose.pose.orientation)


def wrapToPi(angle):
    if angle > m.pi:
        angle -= 2*m.pi
    elif angle < -m.pi:
        angle += 2*m.pi
    return angle


def scanDataCoords():
    coords = []
    for scan in range(len(message.scandata)):
        angle = wrapToPi(message.angles[scan] + message.theta)
        xCoord = message.posx + message.scandata[scan]*m.cos(angle)
        yCoord = message.posy + message.scandata[scan]*m.sin(angle)
        coords.append((xCoord, yCoord))
    message.scanCoords = coords


def get_points_for_visualization(corridor):
    '''Get corridor corners in world frame for visualization.
    '''
    rect = []
    tilt = message.theta
    R = np.array([[np.cos(tilt), -np.sin(tilt)],
                  [np.sin(tilt), np.cos(tilt)]])

    for vertex in corridor.corners:
        vertex_transf = R @ np.array([[vertex[0]], [vertex[1]]]) + \
                        np.array([[message.posx], [message.posy]])
        rect.append(Point(vertex_transf[0], vertex_transf[1], 0))
    return rect


def visualize_rectangle(rect, i, r, g, b):
    '''Visualize corridors.
    '''
    rect_marker = Marker()
    rect_marker.header.stamp = rospy.Time.now()
    rect_marker.header.frame_id = 'odom'
    rect_marker.ns = 'rect'
    rect_marker.id = i
    rect_marker.action = 0
    rect_marker.scale.x = 0.03
    rect_marker.color.r = r
    rect_marker.color.g = g
    rect_marker.color.b = b
    rect_marker.color.a = 1.0
    rect_marker.pose.orientation.w = 1
    rect_marker.lifetime = rospy.Duration(0)
    rect_marker.type = 4  # Line Strip
    rect_marker.points = rect + [rect[0]]

    name = '/rect'
    marker_pub = rospy.Publisher(name, MarkerArray, queue_size=0)
    marker_arr = MarkerArray()

    marker_arr.markers.append(rect_marker)
    marker_pub.publish(marker_arr)


def main():
    global message
    message = messageClass()
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    scan_sub = rospy.Subscriber('/front/scan', LaserScan, scanCallback)
    point_sub = rospy.Subscriber("/front/scan", LaserScan, pointCallback,
                                 queue_size=1)

    rospy.init_node('corridor_node', anonymous=True)
    rate = rospy.Rate(.2)

    print('Building initial corridor')
    corridor_0 = Corridor(width=2., height=2., center=[0, 0],
                          tilt=0, copy=True)
    message.corridors.append(corridor_0)
    corridor_0.corners_world = get_points_for_visualization(corridor_0)
    visualize_rectangle(corridor_0.corners_world, 100, 0.7, 0.7, 0.7)
    best_corridor = [corridor_0]

    print('Corridor node started')
    while not rospy.is_shutdown():
        # Convert laser scan data to point cloud and homogenous coordinates.
        for i, corridor in enumerate(best_corridor):
            corridor.corners_world = get_points_for_visualization(corridor)
            visualize_rectangle(corridor.corners_world, 100+i, 0.7, 0.7, 0.7)

        point_generator = pc2.read_points(message.pointcloud)
        xyh = []
        i = 0
        for point in point_generator:
            i = i + 1
            if i % 4 == 0:  # Only select 1/4th of points.
                xyh.append([point[0], point[1], 1])
        xy = np.array([value for value in xyh]).T

        print('Building corridor')
        best_cor = best_corridor[-1]
        centers = [[0, 0]]
        centers = centers + centers + centers
        tilts = [0, np.pi/6, -np.pi/6]
        best_cor.sweep_corridor(width=.6, height=1.,
                                centers=centers, tilts=tilts, copy=True)

        for i, corridor in enumerate(best_cor.children):
            corridor.grow_all_edges(xy)
            message.corridors.append(corridor)
            corridor.corners_world = get_points_for_visualization(corridor)
            visualize_rectangle(corridor.corners_world, i+1, 0., 0., 0.7)

        next_corridor = sorted(best_cor.children, key=lambda corridor:
                               max([x.x for x in corridor.corners_world]),
                               reverse=True)[:1]

        best_corridor += [next_corridor[0]]

        rate.sleep()


if __name__ == '__main__':
    main()
