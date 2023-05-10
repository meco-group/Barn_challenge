#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mai 10, 2023

@author: bastiaan and mathias
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf2_geometry_msgs import PoseStamped

import math as m

import tf2_ros


def yawFromQuaternion(orientation):
    return m.atan2((2.0*(orientation.w * orientation.z +
                         orientation.x * orientation.y)),
                   (1.0 - 2.0*(orientation.y * orientation.y +
                               orientation.z * orientation.z)))


def odomCallback(data, args):
    '''Transform pose expressed in odometry to map. Publish on topic.

    :param data: pose expressed in odom frame
    :type data: geometry_msgs.PoseStamped

    :param pose_map_pub: publisher for pose2d in map frame
    :type pose_map_pub: rospy.Publisher

    :param tf_buffer: tf buffer
    :type tf_buffer: tf2_ros.tfBuffer
    '''
    pose_map_pub, tf_buffer = args
    pose_odom = PoseStamped(header=data.header, pose=data.pose.pose)
    try:
        # ** It is important to wait for the listener to start listening
        # Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_odom, 'map',
                                                  rospy.Duration(.1))
        output_pose_2D = Pose2D(output_pose_stamped.pose.position.x,
                                output_pose_stamped.pose.position.y,
                                yawFromQuaternion(
                                    output_pose_stamped.pose.orientation))
        pose_map_pub.publish(output_pose_2D)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        raise


def main():
    rospy.init_node('odom_remap', anonymous=True)

    pose_map_pub = rospy.Publisher('/pose_map', Pose2D, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    tf_buffer.lookup_transform('odom', 'map', rospy.Time(0), rospy.Duration(10.))

    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback,
                                (pose_map_pub, tf_buffer))
    print('remapper active')
    rospy.spin()


if __name__ == '__main__':
    main()
