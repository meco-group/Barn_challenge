#!/usr/bin/env python3

import numpy as np
import time
import matplotlib.pyplot as plt
from copy import copy

from corridor_helpers import get_corners, get_center, check_significantly_different


GOAL = (-2, 13)
GOAL_2 = (-2, 11)


class CorridorWorld:
    '''Corridor object for representation of free space between obstacles.
    '''
    DELTA = 1
    FWD = 0
    RGT = 1
    BCK = 2
    LFT = 3

    def __init__(self, width, height, center, tilt=0, parent=None, copy=False):
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

        :param copy: True if you want to make a copy of the original Corridor
        :type copy: bool
        '''
        # Parameters
        self.delta = self.DELTA
        self.height = height
        self.width = width
        self.quality = 0.

        # Family tree
        self.children = []
        self.parent = parent
        if parent:
            self.parent.children.append(self)

        self.W = self._init_W_from_width_height_tilt(
            width, height, *center, tilt)
        self.center = center
        self.growth_center = center.copy()
        self.tilt = tilt
        self.corners = get_corners(self.W)

        # Get individual edge parameter vectors
        self.wf, self.wr, self.wb, self.wl = self.W.T
        self.WF_ind = [-1, 0, 1]
        self.WR_ind = [0, 1, 2]
        self.WB_ind = [1, 2, 3]
        self.WL_ind = [-2, -1, 0]

        # Get hardcopy of original corridor
        if copy:
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


    def add_child_corridor(self, child):
        '''Add child corridor to this corridor and add this 
        corridor as parent to the child
        '''
        self.children.append(child)
        child.parent = self


    def remove_child_corridor(self, child_ind):
        '''Remove the child corridor with given index in the list
        of children and remove this corridor as parent of the child
        '''
        self.children[child_ind].parent = None
        self.children.pop(child_ind)


    def remove_similar_children(self):
        for i in range(len(self.children)-1,-1,-1):
            for j in range(i):
                if not check_significantly_different(self.children[i], self.children[j], 0.1):
                    print("removing too similar child")
                    self.remove_child_corridor(i)
                    break # break from outer loop


    def sort_children(self):
        self.children.sort(key = lambda x : max([x.corners[k][1] for k in range(len(x.corners))]), reverse=True)


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
        t0 = time.time()
        # Check if datapoints are inside the corridor
        for i in range(np.size(datapoints, 1)):
            if np.all(Wtransp @ datapoints[:, i] <= 0):
                t1 = time.time()

                return True
        t1 = time.time()
        self.t_check_inside.append(t1-t0)

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

