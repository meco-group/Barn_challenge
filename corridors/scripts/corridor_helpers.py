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

def check_stuck(parent, child, threshold=0.3):
    '''
        returns a boolean indicating whether the child corridor
        improves enough on the parent corridor

        Returning True means that the child corridor does not improve enough
    '''
    H1 = parent.height/2
    if np.abs(child.growth_center[1] - child.center[1]) < 1.0e-8:
        alpha = 0
    else:
        alpha = -np.arctan((child.growth_center[0]-child.center[0])/(child.growth_center[1]-child.center[1]))   # angle between centerline of the corridor and line between growth-center and center
    centers_dist = np.sqrt(np.power(child.growth_center[0]-child.center[0],2) + np.power(child.growth_center[1]-child.center[1],2))
    H2 = child.height/2 + centers_dist*np.cos(alpha-child.tilt)
    W1 = parent.width/2
    x1 = child.growth_center[0]-parent.center[0]
    y1 = child.growth_center[1]-parent.center[1]
    tilt1 = parent.tilt
    tilt2 = child.tilt

    tilt_critical = np.arctan((W1-x1)/(H1-y1))
    if tilt2-tilt1 < -tilt_critical or tilt2-tilt1 > tilt_critical:
        d_improvement = max(0,H2-(W1-x1)/np.cos(np.pi/2-np.abs(tilt2)+tilt1))
    else:
        d_improvement = max(0,H2-(H1-y1)/np.cos(tilt2-tilt1))

    return d_improvement < threshold

def check_significantly_different(corridor1, corridor2, distance_threshold=0.2, tilt_threshold=np.pi/6):
    '''
        Compute the distance between the 'forward' corners of the
        corridors and compare it with the threshold.

        Returning True means that the corridors are considered to be
        different enough to keep them both
    '''
    corners1 = get_corners(corridor1.W)
    corners2 = get_corners(corridor2.W)

    indx1 = 0
    indx2 = 1

    distance_bool = np.sqrt(np.power(corners1[indx1][0]-corners2[indx1][0], 2) + np.power(corners1[indx1][1]-corners2[indx1][1], 2)) + \
        np.sqrt(np.power(corners1[indx2][0]-corners2[indx2][0], 2) + np.power(corners1[indx2][1]-corners2[indx2][1], 2)) > distance_threshold
    tilt_bool = np.abs(corridor1.tilt - corridor2.tilt) > tilt_threshold

    return distance_bool and tilt_bool

def get_back_track_point(current_corridor, EXPLORE_FULL_CORRIDOR):
    '''
        Given the corridor the robot is currently in, this function
        returns the point to backtrack to from which we can get to another
        branch
    '''
    if EXPLORE_FULL_CORRIDOR:
        # we know this current corridor is a dead end, so we have to start 
        # checking for other options at the parent level
        corridor_to_explore = current_corridor.parent
        previous_point = current_corridor.growth_center

        backtracking_corridors = [current_corridor, corridor_to_explore]

        while not corridor_to_explore is None and len(corridor_to_explore.children) <= 1:
            if corridor_to_explore is None:
                return (None, current_corridor, [])
            
            previous_point = corridor_to_explore.growth_center
            corridor_to_explore = corridor_to_explore.parent

            backtracking_corridors.append(corridor_to_explore)

        if corridor_to_explore is None:
            return (None, current_corridor, [])
        corridor_to_explore.remove_child_corridor(0)
        return (corridor_to_explore.children[0].growth_center, corridor_to_explore.children[0], backtracking_corridors)
    
    else:
        if current_corridor.parent is None:
            return (None, current_corridor, [])
        else:
            parent = current_corridor.parent
            current_corridor.parent.remove_child_corridor(0)
            return (current_corridor.growth_center, parent, [current_corridor, parent])

def check_end_of_corridor_reached(current_corridor, current_pos, threshold=0.3):
    '''
        Compute the distance from the current position to the edge of the corridor
        It is assumed that the current position is inside the current corridor
    '''
   
    top_left = current_corridor.corners[0]
    top_right = current_corridor.corners[1]
    top_center = [(top_left[0]+top_right[0])/2, (top_left[1]+top_right[1])/2]

    # if np.abs(top_right[0] - top_left[0]) < 1.0e-8:
    #     distance_to_end = np.abs(current_pos.posx - top_right[0])
    # else:
    #     a = (top_right[1]-top_left[1])/(top_right[0]-top_left[0])
    #     b = -1.
    #     c = top_left[1] - a
    #     distance_to_end = np.abs(a*current_pos.posx+b*current_pos.posy+c)/np.sqrt(a*a + b*b)

    distance_to_end = np.sqrt((top_center[0]-current_pos.posx)**2 + (top_center[1]-current_pos.posy)**2)

    print("[manager] distance to end of corridor: ", round(distance_to_end, 3))

    # print("[manager] distance to top right: ", np.sqrt((top_right[0]-current_pos.posx)**2 + (top_right[1]-current_pos.posy)**2))

    return distance_to_end < threshold