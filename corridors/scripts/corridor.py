#!/usr/bin/env python3

import numpy as np
import time
import matplotlib.pyplot as plt
from copy import copy

from corridor_helpers import get_corners, get_center, check_significantly_different


GOAL = (-2, 13)
GOAL_2 = (-2, 11)


class Corridor:
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
        # Timing
        self.t_grow_check = []
        self.t_check_inside = []

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
        self.corners_world = []

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
        Nrot = R @ np.array([[1, 0, -1, 0],
                             [0, -1, 0, 1]])
        T0_rot = R @ np.array([[height/2, 0, -height/2, 0],
                               [0, -width/2, 0, width/2]])
        T = T0_rot + np.array([[xcenter], [ycenter]])

        Nrot_T = np.array([Nrot[:, 0] @ T[:, 0], Nrot[:, 1] @ T[:, 1],
                           Nrot[:, 2] @ T[:, 2], Nrot[:, 3] @ T[:, 3]])
        W = np.vstack([Nrot, -Nrot_T])

        return W

    def update_W(self):
        self.W = self._init_W_from_width_height_tilt(self.width, self.height, self.center[0], self.center[1], self.tilt)

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


    def grow_edge(self, datapoints, edge):
        '''Grow specified edge of the corridor.

        :param datapoints: array with 2d homogeneous datapoints
        :type datapoints: numpy.ndarray

        :param edge: edge to grow
        :type edge: int
        '''
        step = self.DELTA
        # Do a first check to see if initial corridor contains datapoints.
        if self.check_inside(datapoints):
            print('discard corridor')
        else:
            # Do maximum 5 big steps in any direction.
            for i in range(5):
                self.W[2, edge] -= step
                self.corners = get_corners(self.W)
                # Continue as long as there are no datapoints inside the grown
                # corridor.
                if (not self.check_inside(datapoints) and
                   not self.check_max_forward()):
                    continue
                # Take some steps back if last grown corridor has a datapoint
                # inside.
                else:
                    while self.check_inside(datapoints):
                        self.W[2, edge] += step/3
                        self.corners = get_corners(self.W)
                    break

        # Recompute the corridorvcorners and center
        self.height = - self.wf[2] - self.wb[2]
        self.width = - self.wr[2] - self.wl[2]
        self.center = self.get_center()

    def grow_all_edges(self, datapoints):
        '''Grow all edges of the corridor.

        :param datapoints: array with 2d homogeneous datapoints
        :type datapoints: numpy.ndarray
        '''
        # print('growing forward')
        self.grow_edge(datapoints, Corridor.FWD)
        # print('growing right')
        self.grow_edge(datapoints, Corridor.RGT)
        # print('growing left')
        self.grow_edge(datapoints, Corridor.LFT)
        print('growing finished')

        # Rotate corridor only if width becomes larger than height
        if self.width > self.height:
            print('rotate')
            self.rotate_corridor()

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

    def check_max_forward(self):
        '''Check if corridor has reached maximum length in forward direction,
        based on the corner positions.
        '''
        max_fwd = GOAL_2[1]
        corner_y = max([x[1] for x in self.corners])
        return corner_y >= max_fwd

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

    def rotate_corridor(self):
        '''Allow rotation of corridor if width would become bigger than length
        by 90 degrees. Rotate left or right depending on initial tilt of
        corridor.
        '''
        self.height, self.width = self.width, self.height
        if self.tilt > 0:
            self.W = self.W[:, [1, 2, 3, 0]]
        else:
            self.W = self.W[:, [3, 0, 1, 2]]

        self.wf, self.wr, self.wb, self.wl = self.W.T

    def sweep_corridor(self, width, height, centers, tilts, copy=False):
        '''Create a new set of Corridors.

        :param width: initial corridor width
        :type width: float
        :param height: initial corridor height
        :type height: float
        :param centers: centerpoints of new corridors (list of np.arrays)
        :type centers: list
        :param tilts: tilts of new corridor (list of floats)
        :type tilts: list
        :param copy: True if you want to make a copy of the original Corridor
        :type copy: bool

        :return: list of Corridors
        :rtype: list
        '''
        corridors = []
        for center, tilt in zip(centers, tilts):
            corridor = Corridor(width=width, height=height,
                                center=center, tilt=tilt,
                                parent=self, copy=copy)
            corridors += [corridor]

        return corridors

    def plot(self, ax=None, color='lightgrey', linestyle='--', linewidth=1):
        '''Plot corridor.'''
        if ax is None:
            plt.figure()
            ax = plt.subplot(111)
        corners = self.corners
        corners.append(corners[0])
        plt, = ax.plot([corner[0] for corner in corners],
                       [corner[1] for corner in corners],
                       color=color, linestyle=linestyle, linewidth=linewidth)
        return plt
