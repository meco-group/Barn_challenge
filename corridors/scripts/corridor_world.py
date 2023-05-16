#!/usr/bin/env python3

import numpy as np
import rospy
from copy import copy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from corridor_helpers import *


GOAL = (-2, 13)


class CorridorWorld:
    '''Corridor object for representation of free space between obstacles.
    '''
    DELTA = 1
    FWD = 0
    RGT = 1
    BCK = 2
    LFT = 3

    def __init__(self, width, height, center, tilt=0,
                 parent=None, copy_in=False):
        '''Constructor.

        :param width: corridor width
        :type width: float

        :param height: corridor height
        :type height: float

        :param center: corridor center
        :type center: numpy.ndarray

        :param tilt: corridor tilt
        :type tilt: float

        :param parent: parent corridor
        :type parent: Corridor

        :param copy_in: True if you want to make a copy of the original
        Corridor
        :type copy_in: bool
        '''
        # Parameters
        self.delta = self.DELTA
        self.height = height
        self.width = width
        self.quality = 0.
        self.fully_explored = False

        # Family tree
        self.children = []
        self.parent = parent
        if parent:
            self.parent.children.append(self)

        self.W = self._init_W_from_width_height_tilt(
            width, height, *center, tilt)
        self.center = center
        self.growth_center = copy(center)
        self.tilt = tilt
        self.corners = get_corners(self.W)

        # Get individual edge parameter vectors
        self.wf, self.wr, self.wb, self.wl = self.W.T
        self.WF_ind = [-1, 0, 1]
        self.WR_ind = [0, 1, 2]
        self.WB_ind = [1, 2, 3]
        self.WL_ind = [-2, -1, 0]

        # Get hardcopy of original corridor
        if copy_in:
            self.original = self.corridor_copy()

    def corridor_copy(self):
        '''Make a copy of the original corridor without reference.
        '''
        return copy(self)

    def _init_W_from_width_height_tilt(self, width, height, xcenter, ycenter,
                                       tilt):
        '''Initialize corridor from width, height and center.

        :param width: corridor width
        :type width: float

        :param height: corridor height
        :type height: float

        :param center: corridor center
        :type center: numpy.ndarray

        :return: W = edge parameter vectors
        :rtype: numpy.ndarray
        '''
        # Construct edge parameter vectors
        R = np.array([[np.cos(tilt), -np.sin(tilt)],
                      [np.sin(tilt), np.cos(tilt)]])
        Nrot = R @ np.array([[0, 1, 0, -1],
                             [1, 0, -1, 0]])
        T0_rot = R @ np.array([[0, width/2, 0, -width/2],
                               [height/2, 0, -height/2, 0]])
        T = T0_rot + np.array([[xcenter], [ycenter]])

        Nrot_T = np.array([Nrot[:, 0] @ T[:, 0], Nrot[:, 1] @ T[:, 1],
                           Nrot[:, 2] @ T[:, 2], Nrot[:, 3] @ T[:, 3]])
        W = np.vstack([Nrot, -Nrot_T])

        return W

    def add_child_corridor(self, child, force_add_child=False, margin=0):
        '''Add child corridor to this corridor and add this
        corridor as parent to the child
        '''
        if force_add_child or self.parent is None or \
            check_significantly_different(self.parent, child, 1.0, 0.0):
            self.children.append(child)
            child.parent = self
            if (max([child.corners[k][1] for k in range(4)]) >
            max([self.corners[k][1] for k in range(4)]) + margin):
                child.quality = 1.0
            else:
                child.quality = 0.1

    def remove_child_corridor(self, child_ind):
        '''Remove the child corridor with given index in the list
        of children and remove this corridor as parent of the child
        '''
        self.children[child_ind].parent = None
        self.children.pop(child_ind)

    def remove_similar_children(self):
        for i in range(len(self.children) - 1, -1, -1):
            for j in range(i):
                if not check_significantly_different(self.children[i],
                                                     self.children[j]):
                    print("Removing too similar child")
                    self.remove_child_corridor(i)
                    break  # break from outer loop

    def has_similar_child(self, potential_child, distance_threshold=None, tilt_threshold=None):
        '''Returns wether or not this corridor has a child that is
        similar to the potential_child corridor that is given
        '''
        similar_found = False
        for child in self.children:
            if not check_significantly_different(child, potential_child, distance_threshold=distance_threshold, tilt_threshold=tilt_threshold):
                similar_found = True
                break

        return similar_found

    def sort_children(self):
        self.children.sort(key=lambda x: max([x.corners[k][1]
                                              for k in range(len(x.corners))]),
                           reverse=True)

    def has_quality_child(self):
        for child in self.children:
            if child.quality >= 0.5:
                return True

        return False

    def check_inside(self, datapoints):
        '''Check if datapoints are inside the corridor.

        :param datapoints: array with 2d homogeneous datapoints, to be checked
            whether they are inside the corridor
        :type datapoints: numpy.ndarray
        :param W: edge parameter vectors
        :type W: numpy.ndarray

        :return: bool array with True if datapoint is inside the corridor
        :rtype: bool
        '''
        Wtransp = self.W.T
        # Check if datapoints are inside the corridor
        for i in range(np.size(datapoints, 1)):
            if np.all(Wtransp @ datapoints[:, i] <= 0):
                return True
        return False

    def get_corners(self):
        '''Get corners of the corridor and store them as self.corners.

        :return: array with corners
        :rtype: numpy.ndarray
        '''
        self.corners = get_corners(self.W)
        return self.corners

    def get_center(self):
        '''Get center of the corridor and store it as self.center.

        :return: center of the corridor
        :rtype: numpy.ndarray
        '''
        self.center = get_center(self.corners)
        return self.center

    def rviz_visualization(self, name, i, r, g, b, lifetime):
        '''Visualize corridors.
        '''
        vertices = []
        for vertex in self.corners:
            vertices.append(Point(vertex[0], vertex[1], 0.05))

        rect_marker = Marker()
        rect_marker.header.stamp = rospy.Time.now()
        rect_marker.header.frame_id = 'map'
        rect_marker.ns = name
        rect_marker.id = i
        rect_marker.action = 0
        rect_marker.scale.x = 0.03
        rect_marker.color.r = r
        rect_marker.color.g = g
        rect_marker.color.b = b
        rect_marker.color.a = 1.0
        rect_marker.pose.orientation.w = 1
        rect_marker.lifetime = rospy.Time(lifetime)
        rect_marker.type = 4  # Line Strip
        rect_marker.points = vertices + [vertices[0]]

        name = '/' + rect_marker.ns
        marker_pub = rospy.Publisher(name, MarkerArray, queue_size=10)
        marker_arr = MarkerArray()

        marker_arr.markers.append(rect_marker)
        marker_pub.publish(marker_arr)
