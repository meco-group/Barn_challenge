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


def check_stuck(parent, child, threshold=0.4):
    '''
    returns a boolean indicating whether the child corridor
    improves enough on the parent corridor

    Returning True means that the child corridor does not improve enough
    '''
    H1 = parent.height/2
    W1 = parent.width/2
    tilt1 = parent.tilt
    tilt2 = child.tilt
    if np.abs(child.growth_center[1] - child.center[1]) < 1.0e-8:
        alpha = 0
    else:
        # rotate the growth center as if the child would have no tilt
        x_tilted = child.center[0] + \
            (child.growth_center[0] - child.center[0])*np.cos(tilt2) + \
            (child.growth_center[1] - child.center[1])*np.sin(tilt2)
        y_tilted = child.center[1] - \
            (child.growth_center[0] - child.center[0])*np.sin(tilt2) + \
            (child.growth_center[1] - child.center[1])*np.cos(tilt2)

        # compute angle between centerline of the corridor and line between
        # growth-center and center
        alpha = - np.arctan2(x_tilted - child.center[0],
                             child.center[1] - y_tilted)

    d_growth_to_child_center = np.sqrt(
        (child.growth_center[0] - child.center[0])**2 +
        (child.growth_center[1] - child.center[1])**2)
    H2 = child.height/2 + d_growth_to_child_center*np.cos(alpha)

    # rotate the growth center as if the parent would have no tilt
    x_tilted = parent.center[0] + \
        (child.growth_center[0] - parent.center[0])*np.cos(tilt1) + \
        (child.growth_center[1] - parent.center[1])*np.sin(tilt1)
    y_tilted = parent.center[1] - \
        (child.growth_center[0] - parent.center[0])*np.sin(tilt1) + \
        (child.growth_center[1] - parent.center[1])*np.cos(tilt1)

    # compute offset of growth_center wrt center of parent
    x1 = parent.center[0] - x_tilted
    y1 = y_tilted - parent.center[1]

    gamma = tilt2 - tilt1

    if gamma >= 0:
        tilt_critical = np.arctan2(W1 - x1, H1 - y1)
        if gamma >= tilt_critical:
            d_improvement = max(0, H2 - (W1 - x1)/np.cos(np.pi/2 - gamma))
        else:
            d_improvement = max(0, H2 - (H1 - y1)/np.cos(gamma))
    else:
        tilt_critical = np.arctan2(-(W1 + x1), H1 - y1)
        if gamma >= tilt_critical:
            d_improvement = max(0, H2 - (H1 - y1)/np.cos(gamma))
        else:
            d_improvement = max(0, H2 - (W1 + x1)/np.cos(gamma + np.pi/2))

    # print(f"[manager] Corridor improvement: {round(d_improvement,2)}")
    return (d_improvement < threshold, d_improvement)


def check_significantly_different(corridor1, corridor2,
                                  distance_threshold=1.7,
                                  tilt_threshold=np.pi/8):
    '''
    Compute the distance between the 'forward' corners of the
    corridors and compare it with the threshold.

    Returning True means that the corridors are considered to be
    different enough to keep them both
    '''
    if distance_threshold is None:
        distance_threshold = 0.4
    if tilt_threshold is None:
        tilt_threshold = np.pi/8
    corners1 = get_corners(corridor1.W)
    corners2 = get_corners(corridor2.W)

    indx1 = 0
    indx2 = 3

    distance_bool = np.sqrt((corners1[indx1][0] - corners2[indx1][0])**2 +
                            (corners1[indx1][1] - corners2[indx1][1])**2) + \
        np.sqrt((corners1[indx2][0] - corners2[indx2][0])**2 +
                (corners1[indx2][1] - corners2[indx2][1])**2) > \
        distance_threshold
    tilt_bool = np.abs(corridor1.tilt - corridor2.tilt) > tilt_threshold

    # print(f"[manager] Similar distance: {round(np.sqrt((corners1[indx1][0] - corners2[indx1][0])**2 + (corners1[indx1][1] - corners2[indx1][1])**2) + np.sqrt((corners1[indx2][0] - corners2[indx2][0])**2 + (corners1[indx2][1] - corners2[indx2][1])**2), 3)} > {0.4}")
    # print(f"[manager] Similar angle: {round(np.abs(corridor1.tilt - corridor2.tilt))} > {round(np.pi/5, 3)}")

    return (distance_bool and tilt_bool)

def dist(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def get_back_track_point(current_corridor, orphanage, EXPLORE_FULL_CORRIDOR):
    '''
    Given the corridor the robot is currently in, this function
    returns the point to backtrack to from which we can get to another
    branch
    '''
    minimal_backtrack_distance = 1.5

    if EXPLORE_FULL_CORRIDOR:
        # we know this current corridor is a dead end, so we have to start
        # checking for other options at the parent level
        corridor_to_explore = current_corridor.parent
        previous_point = current_corridor.growth_center

        backtracking_corridors = [current_corridor, corridor_to_explore]

        while (corridor_to_explore is not None and
                dist(corridor_to_explore.center, current_corridor.center) > minimal_backtrack_distance and
                len(corridor_to_explore.children) <= 1):
            if corridor_to_explore is None:
                return (None, current_corridor, [], orphanage)

            previous_point = corridor_to_explore.growth_center
            corridor_to_explore = corridor_to_explore.parent

            backtracking_corridors.append(corridor_to_explore)

        if corridor_to_explore is None:
            return (None, current_corridor, [], orphanage)
        corridor_to_explore.remove_child_corridor(0)

        for trash_corridor in backtracking_corridors:
            orphanage.add_child_corridor(trash_corridor)

        return (corridor_to_explore.children[0].growth_center,
                corridor_to_explore.children[0],
                backtracking_corridors, orphanage)

    else:
        if current_corridor.parent is None:
            return (None, current_corridor, [], orphanage)
        else:
            parent = current_corridor.parent
            backtracking_corridors = [current_corridor, parent]

            # # Add current corridor to the orphanage
            rejected_child = current_corridor
            parent.remove_child_corridor(0)
            orphanage.add_child_corridor(rejected_child, True)

            # # Place children of the parent that are too similar to children
            # # in the orphanage
            # for i in range(len(parent.children)-1,-1,-1):
            #     if orphanage.has_similar_child(parent.children[i], distance_threshold=2.0, tilt_threshold=-0.1):
            #         parent.remove_child_corridor(i)

            # make sure to backtrack enough generations to make sure that
            # we don't backtrack for only a very small distance
            # Also, backtrack to a corridor that has not yet been fuly explored
            while (parent.parent is not None and 
                (dist(current_corridor.center, parent.center) <= minimal_backtrack_distance or 
                (parent.fully_explored and len(parent.children) == 1))):

                #
                # for i in range(len(parent.children)-1,-1,-1):
                #     if orphanage.has_similar_child(parent.children[i], distance_threshold=2.0, tilt_threshold=-0.1):
                #         parent.remove_child_corridor(i)

                # Move up one generation
                parent = parent.parent
                if parent is not None:
                    backtracking_corridors.append(parent)
                                    # add child to orphanage
                    rejected_child = parent.children[0]
                    parent.remove_child_corridor(0)
                    orphanage.add_child_corridor(rejected_child, True)

            # backtracking_corridors = backtracking_corridors[:-1]
            print(f"\n\n\n\n\nbacktracking_corridors: {backtracking_corridors}")
            print(f"Current corridor: {parent}\n\n\n\n\n")
            print(f"[manager - helper] Found {len(backtracking_corridors)} corridors to backtrack in.")

            return (current_corridor.growth_center, parent,
                    backtracking_corridors, orphanage)

def clear_backtrack_like_corridors(root_corridor, orphanage):
    corridor = root_corridor

    print(f"[manager-helper] orphanage has {len(orphanage.children)} children")
    print("[manager-helper] looping over children")
    # remove children that are close to backtracked corridors
    for i in range(len(corridor.children)-1,-1,-1):
        if orphanage.has_similar_child(corridor.children[i], distance_threshold=2.0, tilt_threshold=-0.1):
            print("[manager] Removing backtracki-like corridor")
            corridor.remove_child_corridor(i)

    print(f"[manager-helper] calling clearing recursively")
    # loop over remaining children and do the same there
    for child in corridor.children:
        clear_backtrack_like_corridors(child, orphanage)

def check_end_of_corridor_reached(current_corridor, current_pos,
                                  threshold=0.3):
    '''
    Compute the distance from the current position to the edge of the corridor
    It is assumed that the current position is inside the current corridor
    '''
    top_left = current_corridor.corners[0]
    top_right = current_corridor.corners[3]
    top_center = [(top_left[0] + top_right[0])/2,
                  (top_left[1] + top_right[1])/2]

    distance_to_end = np.sqrt((top_center[0]-current_pos.posx)**2 +
                              (top_center[1]-current_pos.posy)**2)

    # print("[manager] distance to top right: ", \
    # np.sqrt((top_right[0]-current_pos.posx)**2 +
    # (top_right[1]-current_pos.posy)**2))

    return distance_to_end < threshold
