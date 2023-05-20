#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mai 10, 2023

@author: bastiaan and mathias
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import numpy as np
import math as m

import tf2_ros


def correct_angle_range(angle):
    if angle < 0:
        return correct_angle_range(angle + 2*np.pi)
    elif angle >= 2*np.pi:
        return correct_angle_range(angle - 2*np.pi)
    else:
        return angle


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
    # print('here',pose_odom)
    trans = tf_buffer.lookup_transform('odom', 'map', rospy.Time(0),
                                       rospy.Duration(1.))

    try:
        theta = yawFromQuaternion(trans.transform.rotation)
        rotation = yawFromQuaternion(data.pose.pose.orientation) - theta

        # Create the 2D rotation matrix
        rotation_matrix = np.linalg.inv(
            np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]]))

        # Define a vector or point to be rotated
        point = np.array([data.pose.pose.position.x,
                          data.pose.pose.position.y])  # [x, y] coordinates
        trans = np.array([trans.transform.translation.x,
                          trans.transform.translation.y])

        # Apply the rotation matrix to the point
        rotated_point = np.array(rotation_matrix.dot(point - trans))

        output_pose_2D = Pose2D(rotated_point[0], rotated_point[1],
                                correct_angle_range(rotation))
        pose_map_pub.publish(output_pose_2D)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        raise


def main():
    rospy.init_node('odom_remap', anonymous=True)

    pose_map_pub = rospy.Publisher('/pose_map', Pose2D, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    tf_buffer.lookup_transform('odom', 'map', rospy.Time(0), rospy.Duration(5.))
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback,
                                (pose_map_pub, tf_buffer))
    print('remapper active')
    rospy.spin()


if __name__ == '__main__':
    main()
