from corridor_world import CorridorWorld
import numpy as np
import matplotlib.pyplot as plt

def get_max_radius(posx, posy, theta, corridor, margin):
    '''
        This helper function returns the maximal allowed turning radius for the
        alignment arc such taht the robot does not crash into the wall of the 
        corridor

        However, it is assumed that the corridor does not have a tilt (it is
        aligned with the y-axis)
    '''
    if np.cos(theta) ==1:
        return 100000
    elif theta  > 0:
        return ((corridor.width-2*margin)/2+(posx-corridor.center[0]))/(1-np.cos(theta))
    elif theta < 0:
        return ((corridor.width-2*margin)/2-(posx-corridor.center[0]))/(1-np.cos(theta))
    
def get_max_alignment_radius(posx, posy, theta, corridor, margin = 0):
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
    posx_tilted = corridor.center[0] + (posx-corridor.center[0])*np.cos(corridor.tilt) + (posy-corridor.center[1])*np.sin(corridor.tilt)
    posy_tilted = corridor.center[1] - (posx-corridor.center[0])*np.sin(corridor.tilt) + (posy-corridor.center[1])*np.cos(corridor.tilt)

    # Call the helper to get the maximum radius
    return (get_max_radius(posx_tilted, posy_tilted, theta-corridor.tilt, corridor, margin), posx_tilted, posy_tilted)

# # make a corridor
# c = CorridorWorld(width=0.5, height=1.0, tilt=0.2, center=[0.0,0.0])

# plt.figure()
# ax = plt.gca()

# for posx in np.linspace(-0.18, 0.24, 10):
#     # posx = 0.2
#     posy = -0.2
#     theta = np.pi/4

#     ax.set_aspect('equal')
#     plt.plot([c.corners[k][0] for k in range(len(c.corners))]+[c.corners[0][0]], \
#          [c.corners[k][1] for k in range(len(c.corners))]+[c.corners[0][1]])

#     plt.plot(posx, posy, 'o')

#     (R, posx_tilted, posy_tilted) = get_max_alignment_radius(posx, posy, theta, c)

#     center = (posx_tilted+R*np.cos(theta-c.tilt)*np.sign(theta-c.tilt), posy_tilted+R*np.sin(np.abs(theta-c.tilt)))
#     center_tilted = (c.center[0] + (center[0]-c.center[0])*np.cos(c.tilt) - \
#                      (center[1]-c.center[1])*np.sin(c.tilt),
#                      c.center[1] + (center[0]-c.center[0])*np.sin(c.tilt) + \
#                         (center[1]-c.center[1])*np.cos(c.tilt))

#     ax.add_patch(plt.Circle(center_tilted, R, fill=False))

# plt.figure()
# ax = plt.gca()

# for posx in np.linspace(-0.18, 0.24, 10):
#     # posx = 0.2
#     posy = -0.2
#     theta = -np.pi/4

#     ax.set_aspect('equal')
#     plt.plot([c.corners[k][0] for k in range(len(c.corners))]+[c.corners[0][0]], \
#          [c.corners[k][1] for k in range(len(c.corners))]+[c.corners[0][1]])

#     plt.plot(posx, posy, 'o')

#     (R, posx_tilted, posy_tilted) = get_max_alignment_radius(posx, posy, theta, c)

#     center = (posx_tilted+R*np.cos(theta-c.tilt)*np.sign(theta-c.tilt), posy_tilted+R*np.sin(np.abs(theta-c.tilt)))
#     center_tilted = (c.center[0] + (center[0]-c.center[0])*np.cos(c.tilt) - \
#                      (center[1]-c.center[1])*np.sin(c.tilt),
#                      c.center[1] + (center[0]-c.center[0])*np.sin(c.tilt) + \
#                         (center[1]-c.center[1])*np.cos(c.tilt))

#     ax.add_patch(plt.Circle(center_tilted, R, fill=False))

# plt.show()
