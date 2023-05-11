import numpy as np


def get_max_radius(posx, posy, theta, corridor, margin):
    '''
    This helper function returns the maximal allowed turning radius for the
    alignment arc such that the robot does not crash into the wall of the
    corridor. However, it is assumed that the corridor does not have a tilt
    (it is aligned with the y-axis)
    '''
    if np.cos(theta) == 1 or np.abs(theta) <= 1e-3:
        radius = 100000
    elif theta > 0:
        radius = ((corridor.width/2 - margin) +
                  (posx - corridor.center[0]))/(1 - np.cos(theta))
    elif theta < 0:
        radius = ((corridor.width/2 - margin) -
                  (posx - corridor.center[0]))/(1 - np.cos(theta))

    return radius


def get_max_alignment_radius(posx, posy, theta, corridor, margin=0):
    '''
    This function returns the maximum allowed turning radius for the
    alignment arc such that the robot does not crash into the wall of the
    corridor

    posx:     x-coordinate (in the world) of the robot
    posy:     y-coordinate (in the world) of the robot
    theta:    heading angle of the robot w.r.t the y-axis!
    corridor: corridor defined in the world with a tilt relative to the
                y-axis!
    margin:   safety margin for corridor width
    '''
    # First, transform the robot position to pretend that the corridor has no
    # tilt
    posx_tilted = corridor.center[0] + \
        (posx - corridor.center[0])*np.cos(corridor.tilt) + \
        (posy - corridor.center[1])*np.sin(corridor.tilt)
    posy_tilted = corridor.center[1] - \
        (posx - corridor.center[0])*np.sin(corridor.tilt) + \
        (posy - corridor.center[1])*np.cos(corridor.tilt)

    # Call the helper to get the maximum radius
    max_radius = get_max_radius(posx_tilted, posy_tilted,
                                theta - corridor.tilt, corridor, margin)

    return max_radius, posx_tilted, posy_tilted
