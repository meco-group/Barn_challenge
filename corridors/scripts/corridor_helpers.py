#!/usr/bin/env python3

import numpy as np


def get_intersection(w1, w2):
    '''Use Cramer's rule to solve the system of equations
    ``w1.T @ ph = 0 ; w2.T @ ph = 0;`` for p, where ph = [p; 1]

    :param np.array w1: line parameter vector of shape [w0, w1, w2]
    :param np.array w2: line parameter vector of shape [w0, w1, w2]

    :return: intersection of w1 and w2
    :rtype: np.array
    '''
    return np.array(
        [(w1[1]*w2[2] - w1[2]*w2[1])/(w1[0]*w2[1] - w1[1]*w2[0]),
            (w1[2]*w2[0] - w1[0]*w2[2])/(w1[0]*w2[1] - w1[1]*w2[0])])


def get_corners(W):
    '''Get corners of the corridor.

    :return: corners of the corridor
    :rtype: list
    '''
    # Construct corners
    corners = []

    # First corner double to make the visualization a full polygon.
    W = np.hstack((W, W[:, [0]]))

    for i in range(len(W.T)-1):
        w1 = W[:, i]
        w2 = W[:, i+1]
        corners.append(get_intersection(w1, w2))

    return corners


def get_center(corners):
    '''Get center of the corridor.

    :param corners: corners of the corridor (list of np.arrays)
    :type corners: list

    :return: center of the corridor
    :rtype: np.array
    '''
    center = (corners[0]+corners[2])/2

    return center
