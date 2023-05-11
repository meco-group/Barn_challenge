#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue April 11, 2023

@author: bastiaan and alex
"""

import rospy
from geometry_msgs.msg import Point, Pose2D
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from visualization_msgs.msg import Marker, MarkerArray
import math as m
import numpy as np
import tf2_ros

from barn_challenge.msg import \
    GoalMsg, CorridorLocalMsg, CorridorLocalListMsg
from corridor import Corridor
from corridor_helpers import *


class messageClass():
    def __init__(self):
        self.scandata = None
        self.ranges = []
        self.free_angles = []
        self.angle_inc = None
        self.angles = []
        self.minind = 0
        self.maxind = 719
        self.angle_min = None
        self.pointcloud = None
        self.scanCoords = []
        self.corridors = []


class lidarData():
    def __init__(self):
        # self.sensor_num = int(720/2)  # Consider only half of the lidar range
        self.sensor_num = 600  # Consider more than half of the lidar range.
        self.lidar_data = np.zeros(self.sensor_num)


class odomData():
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.posz = 0.0
        self.yaw = 0.0

        self.velx = 0.0
        self.rotz = 0.0
        self.theta = 0.0


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


def wrap_to_pi(angle):
    if angle > m.pi:
        angle -= 2*m.pi
    elif angle < -m.pi:
        angle += 2*m.pi
    return angle


def scanCallback(data):
    # lidar_data.lidar_data = np.asarray(data.ranges[(0+179):(719-180)])
    lidar_data.lidar_data = np.asarray(data.ranges[(0+59):(719-60)])
    lidar_data.lidar_data[lidar_data.lidar_data == float('inf')] = 10.


def scanFitterCallback(data):
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
    odom_data.posx = data.x
    odom_data.posy = data.y
    odom_data.yaw = data.theta


def goalPositionCallback(data):
    global goal
    goal = data.goal


def visualizeArrows(angles, rate):
    marker_array = MarkerArray()
    for i in range(len(angles)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'points_arrows'
        marker.lifetime = rospy.Time(rate)
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
        angle = wrap_to_pi(angle)
        x2, y2 = local_to_world(2, 0, angle)
        tip = Point(x2, y2, odom_data.posz)
        marker.points = [tail, tip]
        marker_array.markers.append(marker)
    return marker_array


def main():
    global lidar_data
    global odom_data
    global message
    global goal
    lidar_data = lidarData()
    odom_data = odomData()
    message = messageClass()

    # sensor_range_used = 1/2
    sensor_range_used = 1/1.2
    sensor_span = (3/2)*(np.pi)*sensor_range_used
    sensor_num = lidar_data.sensor_num
    lidar_resolution = sensor_span/sensor_num  # angle resolution
    # sector_num = 6  # number of sectors
    sector_num = 10  # number of sectors
    sector_size = int(sensor_num/sector_num)  # number of points per sector
    free_sectors = 0.0
    last_free_sectors = 0.0
    flag = 0.0

    robot_radius = 0.25
    safety_radius = 0.1
    goal = np.array([0, 10])

    scan_sub_fitter = rospy.Subscriber('/front/scan', LaserScan,
                                       scanFitterCallback)
    scan_sub = rospy.Subscriber('/front/scan', LaserScan, scanCallback)
    odom_sub = rospy.Subscriber('/pose_map', Pose2D, odomCallback)
    point_sub = rospy.Subscriber("/front/scan", LaserScan, pointCallback,
                                 queue_size=1)
    goal_sub = rospy.Subscriber('/goal_position', GoalMsg,
                                goalPositionCallback)
    marker_pub = rospy.Publisher('/angles_marker', MarkerArray, queue_size=10)
    corridor_pub = rospy.Publisher("/corridor", CorridorLocalListMsg,
                                   queue_size=10)
    rospy.init_node('sweeper_fitter', anonymous=True)
    sweeper_fitter_rate = 2
    rate = rospy.Rate(sweeper_fitter_rate)

    print('Building initial corridor')
    corridor_0 = Corridor(width=2., height=2., center=[0, 0],
                          tilt=0, copy=True)
    message.corridors.append(corridor_0)
    corridor_0.get_vertices_for_visualization(
        odom_data.posx, odom_data.posy, odom_data.yaw)

    orig_heading = odom_data.yaw

    best_corridors = [corridor_0]
    last_best_corr = best_corridors[-1]

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    tf_buffer.lookup_transform('odom', 'map', rospy.Time(0), rospy.Duration(10.))

    print('sweeper and fitter ready')
    while not rospy.is_shutdown():
        sectors = np.full((2*sector_num+1), 1.0)
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
                    starting_sector_angle = 0
                    if i == 0:
                        starting_sector_angle = 0
                    else:
                        starting_sector_angle = \
                            (i*lidar_resolution*sector_size -
                             lidar_resolution*sector_size/2)
                    for k in range(sector_size):
                        if x[k] >= x[x_index]:
                            opening_width = opening_width + arc_length
                            # compare if space is enough for a corridor
                            if opening_width > 2*(robot_radius+safety_radius):
                                opening_found = True
                                start = starting_sector_angle + \
                                    lidar_resolution*k
                                end = starting_sector_angle + \
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
                                start = starting_sector_angle + \
                                    lidar_resolution*k
                                end = starting_sector_angle + \
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
                            start_angle = starting_sector_angle
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

        prev_sec = 0.0
        start_angle = 0.0
        end_angle = 0.0
        opening_found = False

        for i in range(sector_num+1):  # loop through shifted sectors
            half_sector = int(sector_size/2)
            this_sector_size = 0
            if i == 0:
                x = lidar_data.lidar_data[0:half_sector]
                this_sector_size = half_sector
            elif i == sector_num:
                x = lidar_data.lidar_data[(i*sector_size) - half_sector:
                                          ((i+1)*sector_size) - sector_size]
                this_sector_size = half_sector
            else:
                x = lidar_data.lidar_data[(i*sector_size) - half_sector:
                                          ((i+1)*sector_size)-half_sector]
                this_sector_size = sector_size
            x_ordered = np.argsort(x)
            for j in range(this_sector_size):  # loop through data on a sector
                x_index = x_ordered[j]
                arc_length = lidar_resolution*x[x_index]
                opening_width = arc_length/2
                opening_found = False
                if x[x_index] >= 0.1:
                    for k in range(this_sector_size):
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
        free_angles = free_angles - sensor_span/2
        free_angles_all = free_angles_all - sensor_span/2

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

        # Convert laser scan data to point cloud and homogenous coordinates.
        point_generator = pc2.read_points(message.pointcloud)
        xyh = []
        i = 0
        for point in point_generator:
            i = i + 1
            if i % 4 == 0:  # Only select 1/4th of points.
                xyh.append([point[0], point[1], 1])
        xy = np.array([value for value in xyh]).T

        print('Building new corridors')
        x, y, theta = odom_data.posx, odom_data.posy, odom_data.yaw
        rotation_angle = orig_heading - np.pi/2
        last_best_corr = best_corridors[-1]

        if True:
            sweep_size = len(free_angles_all)
            centers = sweep_size*[[0, 0]]
            tilts = list(free_angles_all)
            if sweep_size == 1:
                centers = 2*[[0, 0]]
                tilts.append(0.0)
            elif sweep_size == 0:
                centers = 2*[[0, 0]]
                tilts.append(0.0)
                tilts.append(np.pi/3)

            last_best_corr.sweep_corridor(width=.6, height=1.,
                                         centers=centers, tilts=tilts,
                                         copy=True)

            for i, corridor in enumerate(last_best_corr.children):
                corridor.grow_all_edges(xy)
                message.corridors.append(corridor)  # TODO do dit weg
                corridor.get_vertices_for_visualization(x, y, theta)
                # corridor.rviz_visualization('rect_fitter', i, 0.,0.,0.7,
                #                             1/sweeper_fitter_rate)

            a = odom_data.posx - goal[0]
            b = goal[1] - odom_data.posy
            angle_test = m.atan2(a, b) + np.pi/2 - theta
            final_corridor = Corridor(width=1.5, height=1., center=[0, 0],
                                      tilt=angle_test, parent=last_best_corr,
                                      copy=True)
            final_corridor.grow_edge(xy, Corridor.FWD, step_multiplier=3)
            final_corridor.get_vertices_for_visualization(x, y, theta)

            next_best_corrs = sorted(last_best_corr.children,
                                     key=lambda corridor:
                                     max([xs.y for xs in corridor.corners_world]),
                                     reverse=True)[:2]

            best_corridors += [next_best_corrs[0], next_best_corrs[1]]
            for i, corridor in enumerate(next_best_corrs):
                corridor.rviz_visualization('rect_fitter',
                                            i+100, 0.7, 0.7, 0.7,
                                            1/sweeper_fitter_rate)

        # Create corridor message for communication
        corridors_msg = CorridorLocalListMsg()

        for i, corridor in enumerate(next_best_corrs):
            corridor_msg = CorridorLocalMsg()
            corridor_msg.height_local = corridor.height
            corridor_msg.width_local = corridor.width
            corridor_msg.quality_local = corridor.quality
            corridor_msg.center_local = corridor.center
            corridor_msg.growth_center_local = corridor.growth_center
            corridor_msg.tilt_local = corridor.tilt
            corridor_msg.init_pos_global = [x, y, theta]
            corridors_msg.len = i + 1

            xy_corners = []
            for xy in corridor.corners_world:
                xy_corners.append(xy.x)
                xy_corners.append(xy.y)
            corridor_msg.corners_local = xy_corners

            corridors_msg.corridors.append(corridor_msg)

        corridor_pub.publish(corridors_msg)

        marker_array = visualizeArrows(free_angles_all,
                                        1/sweeper_fitter_rate)
        marker_pub.publish(marker_array)

        rate.sleep()


if __name__ == '__main__':
    main()
