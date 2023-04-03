#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 15:38:15 2023

@author: bastiaan
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker,MarkerArray
import laser_geometry.laser_geometry as lg
import math as m
import numpy as np

from corridor import Corridor

class messageClass():
	def __init__(self):
		self.goalx = None
		self.goaly = None
		self.velx = 0.0
		self.velz = 0.0
		self.target_velocity = 2.0
		self.posx = 0.0
		self.posy = 0.0
		self.theta = 0.0
		self.scandata = None
		self.ranges = []
		self.angle_inc = None
		self.scanCoords = []
		self.angles = []
		self.gapGoal = None
		self.buffer = 0.5
		self.footpr = 0.8
		self.minind = 0
		self.maxind = 719
		self.angle_min = None
		self.pointcloud = None

def distance(point1,point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)
        
def gapGoalCallback(data):
    message.gapGoal = data.data[:2]

def yawFromQuaternion(orientation):
    return m.atan2((2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)),
                  (1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)))

def scanCallback(data):
    message.scandata = np.asarray(data.ranges[message.minind:message.maxind])
    message.angle_inc = data.angle_increment
    if not len(message.angles):
        message.angles = np.array([data.angle_min + i * data.angle_increment for i in range(message.minind,message.maxind)])
        message.angle_min = data.angle_min

lp = lg.LaserProjection()
def pointCallback(data):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(data)

    message.pointcloud = pc2_msg
    # now we can do something with the PointCloud2 for example:
    # publish it
    # pc_pub.publish(pc2_msg)
    
    # convert it to a generator of the individual points
    # point_generator = pc2.read_points(pc2_msg)
    


def odomCallback(data):
	message.velx = data.twist.twist.linear.x
	message.velz = data.twist.twist.angular.z
	message.posx = data.pose.pose.position.x
	message.posy = data.pose.pose.position.y
	message.theta = yawFromQuaternion(data.pose.pose.orientation)
    
def wrapToPi(angle):
    if angle>m.pi:
        angle-=2*m.pi
    elif angle<-m.pi:
        angle+=2*m.pi
    return angle
    
def scanDataCoords():
    coords = []
    for scan in range(len(message.scandata)):
        angle = wrapToPi(message.angles[scan] + message.theta)
        xCoord = message.posx + message.scandata[scan]*m.cos(angle)
        yCoord = message.posy + message.scandata[scan]*m.sin(angle)
        coords.append((xCoord,yCoord))
    message.scanCoords = coords

def first_rectangle(rect):
    rect_marker = Marker()
    rect_marker.header.stamp = rospy.Time.now()
    rect_marker.header.frame_id = "odom"
    rect_marker.ns = "rect0"
    rect_marker.id = 1
    rect_marker.action = 0
    rect_marker.scale.x = 0.03
    rect_marker.color.r = 0.
    rect_marker.color.g = 0.
    rect_marker.color.b = 0.7
    rect_marker.color.a = 1.0
    rect_marker.pose.orientation.w = 1
    rect_marker.lifetime = rospy.Duration(0)
    rect_marker.type = 4  # Line Strip
    rect_marker.points = rect + [rect[0]]

    marker_pub = rospy.Publisher('/rect0', MarkerArray, queue_size=0)
    marker_arr = MarkerArray()

    marker_arr.markers.append(rect_marker)
    marker_pub.publish(marker_arr)


def main():
    global message
    message = messageClass()
    # gaps = rospy.Subscriber('/gapGoal', Float32MultiArray, gapGoalCallback)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    scan_sub = rospy.Subscriber('/front/scan', LaserScan, scanCallback)
    vel_Pub  = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    pc_Pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
    point_sub = rospy.Subscriber("/front/scan", LaserScan, pointCallback, queue_size=1)
    
    rospy.init_node('old_mcdonalds_local', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    
    maxSpeed = 1
    minSpeed = 0.1
    maxTurn = m.pi/2
    isDone = False
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    message.goalx = message.posx
    message.goaly = message.posy + 10
    kp = 0.55
    kt = 1.1
    print("I started")
    flag = True
    while not rospy.is_shutdown():        
        # print('busy')
        # print(message.scandata)
        twist.linear.x = .2
        twist.angular.z = 0

        # localGoal = tuple(message.gapGoal)
        # ang_err = m.atan2(m.sin(theta_d-message.theta),m.cos(theta_d-message.theta))
        # ang_v = np.sign(ang_err) * min(maxTurn,kt*abs(ang_err))
        # twist.angular.z = ang_v
        # # lin_prop = (1-minSpeed)/(1+m.exp(8*(ang_err-m.pi/8)))
        # # twist.linear.x = kp*maxSpeed*lin_prop+minSpeed
        # twist.linear.x = kp*maxSpeed*(1-abs(ang_v)/maxTurn)
        
        # distToGoal = distance((message.goalx,message.goaly),(message.posx,message.posy))

        # planToGoal = distance((message.goalx,message.goaly),(r_goal[0],r_goal[1]))

        # if planToGoal <= 0.5:
        #     kp=.75

        # if distToGoal<0.5:
        #     twist.linear.x = 0
        #     twist.angular.z = 0
        #     print('I stahped')
            # isDone = True
        # print(twist)
        vel_Pub.publish(twist)
        pc_Pub.publish(message.pointcloud)
        
        point_generator = pc2.read_points(message.pointcloud)
        #  we can access a generator in a loop
        xyh = []
        i = 0
        for point in point_generator:
            i = i+1
            if i%4 == 0:
                xyh.append([point[0], point[1], 1])
        
        xy = np.array([value for value in xyh]).T
     

        if flag:
            print('start')
            print(xy)
            start_pose = [0, 0]
            corridor_0 = Corridor(width=.6, height=1, center=start_pose, copy=True)
            corridor_0.grow_all_edges(xy)
            print('stop')

            flag = False
        # print('pos', message.posx, message.posy)
        rect0 = []
        for vertex in corridor_0.corners:
            rect0.append(Point(vertex[0], vertex[1], 0))

        first_rectangle(rect0)
       
        if isDone:
            rospy.signal_shutdown('Goal Reached')
        
        rate.sleep()
                
if __name__ == '__main__':
    main()
