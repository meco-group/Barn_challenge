import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan2, arcsin, linalg, arctan
import numpy as np

from corridor_world import CorridorWorld
from corridor_helpers import get_intersection
from alignment_radius import get_max_alignment_radius


def compute_goal_point(corridor, m):
    '''
    Compute the goal position given a corridor and a margin m (float).
    The goal position is computed as the middle point along the forward face
    of the given corridor, shifted of a quantity equal to m along the
    line connecting the center and the middle point.

    :param corridor: corridor containing goal position
    :type corridor: corridor
    :param m: margin to avoid that the goal position is on a corridor edge
    :type m: float

    :return: goal_pos = x and y coordinates of the goal position
    :rtype: numpy.ndarray
    '''
    height = corridor.height
    tilt = corridor.tilt
    goal_pos = corridor.center + np.array([-(height/2 - m)*sin(tilt),
                                           (height/2 - m)*cos(tilt)])
    return goal_pos


def compute_initial_point(corridor, m):
    '''
    Compute the initial position given a corridor and a margin m (float).
    The initial position is computed as the middle point along the backward
    face of the given corridor, shifted of a quantity equal to m along the
    line connecting the center and the middle point.

    :param corridor: corridor containing initial position
    :type corridor: corridor
    :param m: margin to avoid that the initial position is on a corridor edge
    :type m: float

    :return: initial_pos = x and y coordinates of the initial position
    :rtype: numpy.ndarray
    '''
    height = corridor.height
    tilt = corridor.tilt
    initial_pos = corridor.center + np.array([(height/2 - m)*sin(tilt),
                                              -(height/2 - m)*cos(tilt)])
    return initial_pos


def check_inside_one_point(corridor, point):
    '''Check if one point is inside the corridor. This function is based
    on check_inside

    :param point: array with x and y coordinates of the point to be checked
        whether it is inside the corridor
    :type point: numpy.ndarray
    :param corridor: corridor to be checked whether it contains the point
    :type corridor: corridor

    :return: bool with True if point is inside the corridor
    :rtype: bool
    '''
    Wtransp = corridor.W.T
    hompoint = np.append(point, 1)
    # Check if point is inside the corridor
    if np.all(Wtransp @ hompoint <= 1e-3):
        return True
    return False


def get_corner_point(parent, child):
    '''
    Given a parent and a child corridor, compute the corner point which
    corresponds to the intersection of their edges.

    :param parent: starting corridor
    :type parent: corridor
    :param child: arrival corridor
    :type child: corridor

    :return: x and y coordinates of the corner point
    :rtype: float
    '''
    W_child = child.W
    W_parent = parent.W
    tilt1 = parent.tilt
    tilt2 = child.tilt
    # initialize a np.array containing the corner points candidates
    corner_point = np.empty([0, 2])
    turn_left = False

    # select the correct faces to check for the corner point, depending
    # whether the maneuver is turning right or left
    if tilt2 >= tilt1:  # turn left
        turn_left = True
        select_edges1 = np.array([0, 2, 3])  # check F, B and L faces of child
        select_edges2 = np.array([0, 3])  # check F and L faces of parent
    else:  # turn right
        select_edges1 = np.array([0, 2, 1])  # check F, B and R faces of child
        select_edges2 = np.array([0, 1])  # check F and R faces of parent

    for i in select_edges1:
        for k in select_edges2:
            w1 = W_child[:, i]
            w2 = W_parent[:, k]
            intersection_point = get_intersection(w1, w2)
            if (check_inside_one_point(parent, intersection_point) and
               check_inside_one_point(child, intersection_point)):
                # if turn left and there are more than one candidate points,
                # take the one with lower x coordinate (more on the left)
                if (turn_left and
                   np.all(intersection_point[0] < corner_point[:, 0])):
                    corner_point = np.vstack((intersection_point,
                                              corner_point))
                # if turn right and there are more than one candidate points,
                # take the one with larger x coordinate (more on the right)
                elif (not turn_left and
                      np.all(intersection_point[0] > corner_point[:, 0])):
                    corner_point = np.vstack((intersection_point,
                                              corner_point))
                else:
                    corner_point = np.vstack((corner_point,
                                              intersection_point))

    # you could also just store all the points in corner_point and at the end
    # get the one with minimum/maximum x coordinate whether you are turning
    # left or right
    x_corner = corner_point[0, 0]
    y_corner = corner_point[0, 1]
    return x_corner, y_corner


def plot_trajectory(corridor1, R, x0, y0, xf, yf, x1, y1, x_center1, y_center1,
                    arc_x1, arc_y1, **kwargs):
    corridor2 = kwargs['corridor2'] if 'corridor2' in kwargs else None

    if corridor2 is None:
        # Plot solution
        figure_solution = plt.figure()
        ax = figure_solution.add_subplot(111)
        plt.title('x and y position')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')

        corners = corridor1.corners
        corners.append(corners[0])
        plt.plot([corner[0] for corner in corners],
                 [corner[1] for corner in corners],
                 'k--', linewidth=1)
        # Plot triangle rectangle, just to be sure
        plt.plot([x1, xf], [y1, yf], 'b-', linewidth=0.8)
        plt.plot(x0, y0, 'ro')
        plt.plot(x1, y1, 'ro')
        plt.plot(xf, yf, 'ro')
        plt.plot(arc_x1, arc_y1, 'k')
        # plt.plot([x_center1 + R * cos(angle1)],
        #          [y_center1+R*sin(angle1)], 'r')

        # plt.axis('square')
        ax.set_aspect('equal')
        # plt.legend()
        # plt.show(block=True)


def planner(corridor1, u_bounds, a, b, m, x0, y0, theta0, plot=False, **kwargs):
    corridor2 = kwargs['corridor2'] if 'corridor2' in kwargs else None
    xf = kwargs['xf'] if 'xf' in kwargs else None
    yf = kwargs['yf'] if 'yf' in kwargs else None

    if corridor2 is None:
        man_seq, path, poses = compute_trajectory(corridor1, u_bounds, a, b, m, x0, y0, theta0, plot, xf = xf, yf = yf)
    else:
        # Check whether initial point (x0,y0) is inside corridor2
        if check_inside_one_point(corridor2, np.array([x0,y0])):
            man_seq, path, poses = compute_trajectory(corridor2, u_bounds, a, b, m, x0, y0, theta0, plot, xf = xf, yf = yf)
        else:
            # corridor2_margin = CorridorWorld(corridor2.width-(a+2*m), corridor2.height-2*m, corridor2.center, corridor2.tilt)
            x_corner, y_corner = get_corner_point(corridor1, corridor2)
            # test_point = compute_initial_point(corridor2, 0)
            # x_corner, y_corner = test_point[0], test_point[1]
            psi = arctan2((y_corner - y0),(x_corner - x0)) - pi/2
            UpFRONT = sin(theta0 - psi) >= 0 # TODO: FIX THIS
            if UpFRONT:
                man_seq, path, poses = compute_trajectory(corridor1, u_bounds, a, b, m, x0, y0, theta0, plot, xf = xf, yf = yf, corridor2 = corridor2)
            else:
                # SEMI-BACKTRACKING
                # Create a corridor that resembles corridor1 but is rotated -pi.
                corridor1_back = CorridorWorld(corridor1.width, corridor1.height, corridor1.center, corridor1.tilt - pi)

                # Compute 
                goal_pos1 = compute_goal_point(corridor1_back, 0)
                distance0 = linalg.norm(goal_pos1-[x_corner, y_corner])
                goal_pos2 = compute_goal_point(corridor1_back, 0.8*distance0)

                # Compute maneuver of going backwards
                man_seq1, path1, poses1 = compute_trajectory(corridor1_back, u_bounds, a, b, m, x0, y0, theta0 - pi , plot, xf = goal_pos2[0], yf = goal_pos2[1])
                man_seq1[:,0:1] = -man_seq1[:,0:1]
                orientation = arctan2((path1[-2,1] - path1[-1,1]), path1[-2,0] - path1[-1,0])
                # Compute maneuver from point backwards to corridor2
                man_seq2, path2, poses2 = compute_trajectory(corridor1, u_bounds, a, b, m, path1[-1,0], path1[-1,1], orientation, plot, corridor2 = corridor2)
                # Concatenate path, maneuver and poses
                path = np.vstack((path1,path2))
                man_seq = np.vstack((man_seq1,man_seq2))
                poses = np.vstack((poses1,poses2))

    return man_seq, path, poses


def compute_trajectory(corridor1, u_bounds, a, b, m, x0, y0, theta0, plot, **kwargs):
    '''
    Given two corridors, compute the trajectory the robot shall follow to go from an initial point to a goal point whithin the two corridors 
    in minimum time. 

    :param corridor1: starting corridor
    :type corridor1: corridor
    :param u_bounds: array containing the minimum and maximum forward and angular velocities: u_bounds = [v_min, v_max, omega_min, omega_max]
    :type u_bounds: np.array (4,)
    :param a: length of the robot
    :type a: float
    :param b: width of the robot
    :type b: float
    :param m: margin to avoid colliding with corridor walls when turning
    :type m: float
    :param x0: x coordinate of initial position
    :type x0: float
    :param y0: y coordinate of initial position
    :type y0: float
    :param theta0: current orientation of the robot
    :type theta0: float
    :param plot: boolean to plot the corridors
    :type plot: boolean

    :optional param corridor2: arrival corridor. This parameter is provided in case a second corridor is available
    :type corridor2: corridor
    :optional param xf: x coordinate of goal position
    :type xf: float
    :optional param yf: y coordinate of goal position
    :type yf: float

    :return: maneuver contains the control inputs and time maneuver = [v, omega, time]
    :rtype: np.array
    '''
    #Unpack variables
    v_min = u_bounds[0]
    v_max = u_bounds[1]
    omega_min = u_bounds[2]
    omega_max = u_bounds[3]
    corridor2 = kwargs['corridor2'] if 'corridor2' in kwargs else None

    R = v_max/omega_max
    ## Compute the trajetory in case you have only one corridor
    if corridor2 == None:
        R = min(max(get_max_alignment_radius(x0, y0, theta0-pi/2, corridor1, a/2+m )[0], 1e-3), R)
        # print(f"R = {R} m")
        #initialize the output with two maneuvers
        maneuver_sequence = np.empty((0,3))
        goal_pos = compute_goal_point(corridor1,b) #not best implemention, you compute goal_pos but you might not need it 
        xf = kwargs['xf'] if ('xf' in kwargs and kwargs['xf'] is not None) else goal_pos[0]
        yf = kwargs['yf'] if ('yf' in kwargs and kwargs['yf'] is not None) else goal_pos[1]

        c = sqrt((yf-y0)**2 + (xf-x0)**2) # distance to corridor goal

        # threshold = 0.06 #3.4 degrees
        # threshold = 0.10
        threshold = abs(arctan((corridor1.width/2 - a/2 - m)/c))
        # print(f"** threshold = {threshold}, theta = {abs(arctan((corridor1.width/2 - a/2 - m)/c))}")

        #Compute the angle with respect to the horizontal of line connecting initial and goal pos
        ref_orientation = arctan2(yf-y0,xf-x0)
        #In case the difference between the robot orientation and the angle of the line connecting the 
        #initial and final position is smaller than a threshold, just go straight ahead
        if abs(theta0 - ref_orientation) <= threshold:
            print("[motion_planner] going fwd")
            # c = sqrt((yf-y0)**2 + (xf-x0)**2)
            beta = abs(theta0-ref_orientation)
            hyp = c/cos(beta)

            xf = x0 + hyp * cos(theta0)
            yf = y0 + hyp * sin(theta0)

            v1 = v_max
            t1 = hyp/v1
            maneuver_sequence = np.vstack((maneuver_sequence,np.array([v1, 0, t1])))
            poses_sequence = np.vstack((np.array(([x0,y0, theta0], [x0, y0, theta0], [xf,yf,theta0]))))
            computed_path = np.vstack((np.array(([x0,y0], [x0, y0], [xf,yf]))))

        else:
            #if sqrt((yf-y0)**2 + (xf-x0)**2) >= R:
            ###Turn right
            # print(f"Turn right: difference in angles: {theta0 - (ref_orientation)}, cos(diff): {cos(theta0 - (ref_orientation))}")
            if cos(theta0+pi/2 - ref_orientation) < 0:
                
                #Compute center point of osculating circle
                xc1 = x0 + R * cos(theta0 - pi/2)
                yc1 = y0 + R * sin(theta0 - pi/2)
                # Compute the radius of the initial circle
                a1 = sqrt((xf-xc1)**2 + (yf-yc1)**2)
                #v1= v_max
                #if the final position is within the initial circle
                if a1 <= R: # Rotate on the spot
                    alfa = arctan2((yf-y0),(xf-x0))
                    epsilon = alfa - theta0
                    omega = omega_max if (sin(epsilon) > 0) else omega_min #CHECK ALL THE CASES
                    c = sqrt((yf-y0)**2 + (xf-x0)**2)
                    t1 = abs(epsilon/omega)
                    t2 = c/v_max
                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([0, omega, t1])))
                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([v_max, 0, t2])))
                    arc_x1 = x0
                    arc_y1 = y0
                    x1 = x0
                    y1 = y0

                else:
                    #Compute center point of osculating circle
                    # xc1 = x0 + R * cos(theta0 - pi/2)
                    # yc1 = y0 + R * sin(theta0 - pi/2)

                    #Build triangle rectangle between (x_center, y_center), (xf,yf), (x1,y1)
                    # a1 = sqrt((xf-xc1)**2 + (yf-yc1)**2)
                    c1 = sqrt(abs(a1**2 - R**2)) #lenght line segment # TODO/ check this
                    delta1 = arctan2((yf-yc1),(xf-xc1)) + 2*pi #always positive angle
                    gamma1 = arcsin(c1/a1)
                    x1 = xc1 + R*cos(delta1 + gamma1)
                    y1 = yc1 + R*sin(delta1 + gamma1)
                    chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                    if R > 1e-3:
                        iota1 = 2 * arcsin((chord1/2)/R)
                    else:
                        iota1 = abs(theta0 - corridor1.tilt)
                    epsilon1 = arctan2((y0-yc1),(x0-xc1)) + 2*pi #always positive angle

                    arc_x1 = xc1+R*cos(linspace(epsilon1, epsilon1 - iota1, 100))
                    arc_y1 = yc1+R*sin(linspace(epsilon1, epsilon1 - iota1, 100))
                    v1 = R *abs(omega_min)

                    if R > 1e-3:
                        t1 = R * iota1/ v1
                    else: 
                        t1 = iota1 / abs(omega_min)
                        v1 = 0
                    t2 = c1 / v_max

                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([v1, omega_min, t1])))
                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([v_max, 0, t2])))

            else:
                #Compute center point of osculating circle
                xc1 = x0 + R * cos(theta0 + pi/2)
                yc1 = y0 + R * sin(theta0 + pi/2)
                #Compute the radius of the initial circle
                a1 = sqrt((xf-xc1)**2 + (yf-yc1)**2)
                #if the final position is within the initial circle
                if a1 <= R:
                    alfa = arctan2((yf-y0),(xf-x0))
                    epsilon = alfa - theta0
                    omega = omega_max if (sin(epsilon) > 0) else omega_min #CHECK ALL THE CASES
                    c = sqrt((yf-y0)**2 + (xf-x0)**2)
                    t1 = abs(epsilon/omega)
                    t2 = c/v_max
                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([0, omega, t1])))
                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([v_max, 0, t2])))
                    arc_x1 = x0
                    arc_y1 = y0
                    x1 = x0
                    y1 = y0
                else:
                    #Compute center point of osculating circle
                    # xc1 = x0 + R * cos(theta0 + pi/2)
                    # yc1 = y0 + R * sin(theta0 + pi/2)

                    #Build triangle rectangle between (x_center, y_center), (xf,yf), (x1,y1)
                    # a1 = sqrt((xf-xc1)**2 + (yf-yc1)**2)
                    c1 = sqrt(a1**2 - R**2) #lenght line segment
                    delta1 = arctan2((yf-yc1),(xf-xc1)) + 2*pi #always positive angle
                    gamma1 = arcsin(c1/a1)
                    x1 = xc1 + R*cos(delta1 - gamma1)                
                    y1 = yc1 + R*sin(delta1 - gamma1)
                    chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                    if R > 1e-3:
                        iota1 = 2 * arcsin((chord1/2)/R)
                    else:
                        iota1 = abs(theta0 - corridor1.tilt)
                    epsilon1 = arctan2((y0-yc1),(x0-xc1)) + 2*pi #always positive angle

                    arc_x1 = xc1+R*cos(linspace(epsilon1, epsilon1 + iota1, 100))
                    arc_y1 = yc1+R*sin(linspace(epsilon1, epsilon1 + iota1, 100))
                    v1 = R * omega_max

                    if R > 1e-3:
                        t1 = R * iota1/ v1
                    else: 
                        t1 = iota1 / omega_max
                        v1 = 0
                    t2 = c1 / v_max

                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([v1, omega_max, t1])))
                    maneuver_sequence = np.vstack((maneuver_sequence,np.array([v_max, 0, t2])))

            theta1 = arctan2((yf-y1),(xf-x1))
            thetaf = theta1
            poses_sequence = np.vstack((np.array(([x0,y0, theta0], [x1, y1, theta1], [xf,yf,thetaf]))))

            computed_path = np.vstack((
                np.array([arc_x1,arc_y1]).T, 
                [x1,y1], 
                [xf,yf],
            ))

            if plot:
                plot_trajectory(corridor1, R, x0, y0, xf, yf, x1, y1, xc1, yc1, arc_x1, arc_y1)
            # else:
            #     maneuver_sequence = np.empty((0,3))
            #     computed_path = np.empty((0,2))
            #     poses_sequence = np.array(([x0,y0, theta0]))

    ### Two corridors
    else:
        goal_pos = compute_goal_point(corridor2,b)
        xf = kwargs['xf'] if ('xf' in kwargs and kwargs['xf'] is not None) else goal_pos[0]
        yf = kwargs['yf'] if ('yf' in kwargs and kwargs['yf'] is not None) else goal_pos[1]

        R1 = min(max(get_max_alignment_radius(x0, y0, theta0-pi/2, corridor1, m+a/2)[0], 1e-3), R)  

        tilt1 = corridor1.tilt
        tilt2 = corridor2.tilt

        turn_left = False
        go_straight = False
        turn_right = False
        #Turn left or turn right
        if tilt2 > tilt1:
            turn_left = True
        elif tilt2 == tilt1:
            go_straight
        else:
            turn_right = True
    
        if turn_left:
            maneuver_sequence = np.empty((4,3))
            #Compute the center coordinates
            x_corner, y_corner = get_corner_point(corridor1, corridor2)
            tilt1 = corridor1.tilt
            tilt2 = corridor2.tilt
            #Compute the coordinates of the center of circle 2
            #angle = (pi - arccos(cos(tilt1)*cos(tilt2)+sin(tilt1)*sin(tilt2)))/2
            angle = (pi - (tilt2-tilt1))/2
            xc2 = x_corner + (R-a/2-m)*cos(pi/2 + tilt2 + angle)
            yc2 = y_corner + (R-a/2-m)*sin(pi/2 + tilt2 + angle)
            #Compute the radius of the second circle
            R_max = sqrt((yf-yc2)**2+(xf-xc2)**2)
            v2 = v_max
            if R_max <= R:
                R = R_max
                v2 = R * omega_max
            # CASE 1: Turn left-left
            # if theta0 <= pi/2 or theta0 >= 3*pi/2:
            if cos(theta0 - corridor1.tilt) > 0:

                #Compute the coordinates of the center of circle 1
                xc1 = x0 + R1 * cos(theta0 + pi/2)
                yc1 = y0 + R1 * sin(theta0 + pi/2)

                c1 = sqrt((yc2 - yc1)**2 + (xc2 - xc1)**2)
                delta1 = arctan2((yc2 - yc1), (xc2-xc1))
                epsilon1 = delta1 - pi/2
                zeta1 = arctan2((y0 - yc1),(x0 - xc1)) + 2 * pi
                x1 = xc1 + R1* cos(epsilon1)
                y1 = yc1 + R1 * sin(epsilon1)
                chord1 = sqrt((x1 - x0)**2 + (y1-y0)**2)
                iota1 = 2 * arcsin((chord1/2)/R1)
                x2 = x1 + c1 * cos(epsilon1 + pi/2)
                y2 = y1 + c1 * sin(epsilon1 + pi/2)

                #If the final position is inside the second circle, stop, turn in place and reach the final position through a straight line primitive
                if sqrt((yf-yc2)**2 + (xf-xc2)**2) <= R:
                    c2 = sqrt((yf-y2)**2 + (xf-x2)**2)
                    alfa2 = arctan2((yf-y2),(xf-x2))
                    theta2 = arctan2((y2-y1),(x2-x1))
                    epsilon2 = alfa2 - theta2
                    omega3 = omega_max if (sin(epsilon2) > 0) else omega_min #CHECK ALL THE CASES
                    v2 = 0
                    t3 = abs(epsilon2/omega3)
                    arc_x2 = x2
                    arc_y2 = y2
                    x3 = x2
                    y3 = y2
                else:
                    epsilon2 = arctan2((y2 - yc2), (x2 - xc2)) + 2 * pi
                    delta2 = arctan2((yc2-yf),(xc2-xf))
                    a2 = sqrt((yf-yc2)**2+(xf-xc2)**2)
                    c2 = sqrt(a2**2 - R**2)
                    beta2 = arcsin(R/a2)
                    eta2 = delta2 + beta2
                    x3 = xf + c2 * cos(eta2)
                    y3 = yf + c2 * sin(eta2)
                    chord2 = sqrt((x3-x2)**2 + (y3-y2)**2)
                    iota2 = 2*arcsin((chord2/2)/R)
                    t3 = R*iota2/v2
                    omega3 = omega_max
                    arc_x2 = xc2 + R * cos(linspace(epsilon2, epsilon2 + iota2, 100))
                    arc_y2 = yc2 + R * sin(linspace(epsilon2, epsilon2 + iota2, 100))

                #Compute the primitives
                v1 = R1 * omega_max
                if R1 > 1e-3:
                    t1 = R1 * iota1 /v1
                else: 
                    t1 = iota1 / omega_max
                t2 = c1/v_max
                #t3 is computed previusly, depends whether the final point is insied second circle
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v1, omega_max, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v2, omega3, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

                arc_x1 = xc1 + R1 * cos(linspace(zeta1, zeta1 + iota1, 100))
                arc_y1 = yc1 + R1 * sin(linspace(zeta1, zeta1 + iota1, 100))

            #CASE 2: Turn right-left
            else:
                xc1 = x0 + R1 * cos(theta0 - pi/2)
                yc1 = y0 + R1 * sin(theta0 - pi/2)
                new_case = False
                #Check whether the first and second circles are overlapping
                if sqrt((yc2-yc1)**2+(xc2-xc1)**2) <= (R + R1):
                    ipo = sqrt((xc2-x0)**2 + (yc2-y0)**2)
                    if ipo <= R:
                        new_case = True
                    c1 = sqrt((ipo)**2-R**2)
                    beta = arcsin(R/ipo)
                    delta = arctan2((yc2-y0),(xc2-x0))
                    alfa = delta - beta
                    epsilon1 = alfa - theta0
                    omega1 = omega_max if (sin(epsilon1) > 0) else omega_min #CHECK ALL THE CASES
                    v1 = 0
                    t1 = abs(epsilon1/omega1)
                    arc_x1 = x0
                    arc_y1 = y0
                    x1 = x0
                    y1 = y0
                    x2 = x1 + c1 * cos(alfa)
                    y2 = y1 + c1 * sin(alfa)
                    t1 = abs(epsilon1/omega1)
                    
                else:
                    a1 = sqrt((yc2-yc1)**2 + (xc2-xc1)**2)/2
                    c1 = sqrt(a1**2-R1**2)
                    delta1 = arctan2((yc2-yc1),(xc2-xc1))
                    gamma1 = arcsin(c1/a1)
                    x1 = xc1 + R1 * cos(delta1 + gamma1)
                    y1 = yc1 + R1 * sin(delta1 + gamma1)
                    epsilon1 = arctan2((yc1-y1),(xc1-x1))
                    x2 = x1 + 2 * c1 * cos(epsilon1 + pi/2)
                    y2 = y1 + 2 * c1 * sin(epsilon1 + pi/2)
                    eta1 = arctan2((y0-yc1),(x0-xc1)) + 2*pi
                    chord1 = sqrt((y1-y0)**2 + (x1 - x0)**2)
                    if R1 > 1e-3:
                        iota1 = 2 * arcsin((chord1/2)/R1)
                    else:
                        iota1 = abs(theta0 - corridor1.tilt)       
                    arc_x1 = xc1 + R1 * cos(linspace(eta1, eta1 - iota1, 100))
                    arc_y1 = yc1 + R1 * sin(linspace(eta1, eta1 - iota1, 100))
                    v1 = R1 * abs(omega_min)
                    if R1 > 1e-3:
                        t1 = R1 * iota1 / v1
                    else: 
                        t1 = iota1 / abs(omega_min)
                    c1 = 2* c1
                    omega1 = omega_min

                #If the final position is inside the second circle, stop, turn in place and reach the final position through a straight line primitive
                if sqrt((yf-yc2)**2 + (xf-xc2)**2) <= R and not(new_case):
                    c2 = sqrt((yf-y2)**2 + (xf-x2)**2)
                    alfa2 = arctan2((yf-y2),(xf-x2))
                    theta2 = arctan2((y2-y1),(x2-x1))
                    epsilon2 = alfa2 - theta2
                    omega3 = omega_max if (sin(epsilon2) > 0) else omega_min #CHECK ALL THE CASES
                    v2 = 0
                    t3 = abs(epsilon2/omega3)
                    arc_x2 = x2
                    arc_y2 = y2
                    x3 = x2
                    y3 = y2
                elif not(new_case):
                    delta2 = arctan2((yc2-yf),(xc2-xf))
                    a2 = sqrt((yc2-yf)**2 + (xc2-xf)**2)
                    c2 = sqrt(a2**2 - R**2)
                    beta2 = arcsin(R/a2)
                    x3 = xf + c2 * cos(delta2 + 2*pi + beta2)
                    y3 = yf + c2 * sin(delta2 + 2*pi + beta2)
                    eta2 = arctan2((y2-yc2),(x2-xc2))+2*pi
                    chord2 = sqrt((y3-y2)**2 + (x3-x2)**2)
                    iota2 = 2 * arcsin((chord2/2)/R)
                    t3 = R*iota2/v2
                    omega3 = omega_max
                    arc_x2 = xc2 + R * cos(linspace(eta2, eta2 + iota2, 100))
                    arc_y2 = yc2 + R * sin(linspace(eta2, eta2 + iota2, 100))
                else:
                    #If the initial position is inside the second circle
                    delta = arctan2((yc2-y0),(xc2-x0))
                    epsilon1 = delta - theta0
                    omega1 = omega_max if (sin(epsilon1) > 0) else omega_min #CHECK ALL THE CASES
                    v1 = 0
                    t1 = abs(epsilon1/omega1)
                    arc_x1 = x0
                    arc_y1 = y0
                    x1 = x0
                    y1 = y0
                    c1 = sqrt((xc2-x0)**2 - (yc2-y0)**2)
                    x2 = x1 + c1 * cos(alfa)
                    y2 = y1 + c1 * sin(alfa)

                    delta2 = arctan2((yf-y2),(xf-x2))
                    epsilon2 = delta2 - delta 
                    omega3 = omega_max if (sin(epsilon2) > 0) else omega_min #CHECK ALL THE CASES
                    v2 = 0
                    t3 = abs(epsilon2/omega3)
                    arc_x2 = x2
                    arc_y2 = y2
                    x3 = x2
                    y3 = y2
                    c2 = sqrt((xf-xc2)**2 - (yf-yc2)**2)
                    #xf = x1 + c1 * cos(alfa)
                    #y2 = y1 + c1 * sin(alfa)
 
                t2 = c1 / v_max
                #t3 is computed previously
                t4 = c2 / v_max

                maneuver_sequence[0,:] = np.array([v1, omega1, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v2, omega3, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

            angle_complete_circle = linspace(0,2*pi,100)

        elif turn_right:
            maneuver_sequence = np.empty((4,3))
            #Compute the center coordinates
            x_corner, y_corner = get_corner_point(corridor1, corridor2)
            tilt1 = corridor1.tilt
            tilt2 = corridor2.tilt
            #Compute the coordinates of the center of circle 2
            #angle = (pi - arccos(cos(tilt1)*cos(tilt2)+sin(tilt1)*sin(tilt2)))/2
            angle = (pi - abs(tilt2-tilt1))/2
            xc2 = x_corner + (R-a/2-m)*cos(pi/2 + tilt2 - angle)
            yc2 = y_corner + (R-a/2-m)*sin(pi/2 + tilt2 - angle)
            #Compute the radius of the second circle
            R_max = sqrt((yf-yc2)**2+(xf-xc2)**2)
            v2 = v_max
            if R_max <= R:
                R = R_max
                v2 = R * abs(omega_min)
            #CASE 3:Turn left-right
            # if theta0 <= pi/2 or theta0 >= 3*pi/2: turn left-right
            if cos(theta0 - corridor1.tilt) > 0:
                xc1 = x0 + R1 * cos(theta0 + pi/2)
                yc1 = y0 + R1 * sin(theta0 + pi/2)
                new_case = False
                #Check whether the first and second circles are overlapping
                if sqrt((yc2-yc1)**2+(xc2-xc1)**2) <= (R + R1):
                    ipo = sqrt((xc2-x0)**2 + (yc2-y0)**2)
                    if ipo <= R:
                        new_case = True
                    c1 = sqrt((ipo)**2-R**2)
                    beta = arcsin(R/ipo)
                    delta = arctan2((yc2-y0),(xc2-x0))
                    alfa = delta + beta
                    epsilon1 = alfa - theta0
                    omega1 = omega_max if (sin(epsilon1) > 0) else omega_min #CHECK ALL THE CASES
                    v1 = 0
                    t1 = abs(epsilon1/omega1)
                    arc_x1 = x0
                    arc_y1 = y0
                    x1 = x0
                    y1 = y0
                    x2 = x1 + c1 * cos(alfa)
                    y2 = y1 + c1 * sin(alfa)
                    t1 = abs(epsilon/omega1)
                else:
                    a1 = sqrt((yc2-yc1)**2 + (xc2-xc1)**2)/2
                    c1 = sqrt(a1**2 - R1**2) # TODO: NaN could come from here
                    delta1 = arctan2((y0-yc1),(x0-xc1)) + 2*pi #always positive
                    epsilon1 = arctan2((yc2-yc1),(xc2-xc1)) + 2*pi #always positive
                    gamma1 = arcsin(c1/a1)

                    eta1 = epsilon1 - gamma1
                    x1 = xc1 + R1*cos(eta1)
                    y1 = yc1 + R1*sin(eta1)
                    eta1 = arctan2((yc1-y1),(xc1-x1))
                    x2 = x1+2*c1*cos(eta1 - pi/2)
                    y2 = y1 + 2*c1*sin(eta1 - pi/2)
                    chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                    if R1 > 1e-3:
                        iota1 = 2*arcsin((chord1/2)/R1)
                    else:
                        iota1 = abs(theta0 - corridor1.tilt)
                    
                    v1 = R1 * omega_max
                    omega1 = omega_max
                    c1 = 2*c1
                    if R1 > 1e-3:
                        t1 = R1 * iota1 /v1
                    else: 
                        t1 = iota1 / omega_max
                    arc_x1 = xc1 + R1 * cos(linspace(delta1, delta1 + iota1,100))
                    arc_y1 = yc1 + R1 * sin(linspace(delta1, delta1 + iota1,100))
                #If the final position is inside the second circle, stop, turn in place and reach the final position through a straight line primitive
                if sqrt((yf-yc2)**2 + (xf-xc2)**2) <= R and not(new_case):
                    c2 = sqrt((yf-y2)**2 + (xf-x2)**2)
                    alfa2 = arctan2((yf-y2),(xf-x2))
                    theta2 = arctan2((y2-y1),(x2-x1))
                    epsilon2 = alfa2 - theta2
                    omega3 = omega_max if (sin(epsilon2) > 0) else omega_min #CHECK ALL THE CASES
                    v2 = 0
                    t3 = abs(epsilon2/omega3)
                    arc_x2 = x2
                    arc_y2 = y2
                    x3 = x2
                    y3 = y2
                elif not(new_case):
                    delta2 = arctan2((y2-yc2),(x2-xc2)) +2*pi #always positive
                    epsilon2 = arctan2((yc2-yf),(xc2-xf))
                    a2 = sqrt((yf-yc2)**2+(xf-xc2)**2)
                    c2 = sqrt(a2**2-R**2)
                    beta2 = arcsin(R/a2)
                    x3 = xf + c2*cos(epsilon2 - beta2)
                    y3 = yf + c2*sin(epsilon2-beta2)
                    eta2 = arctan2((y3-yc2),(x3-xc2)) +2*pi #always positive
                    omega3 = omega_min
                    chord2 = sqrt((x3-x2)**2+(y3-y2)**2)
                    # iota2 = 2*arcsin((chord2/2)/R)
                    if R > 1e-3:
                        iota2 = 2*arcsin((chord2/2)/R)
                    else:
                        iota2 = abs(corridor1.tilt - corridor2.tilt)
                    t3 = R*iota2/v2
                    arc_x2 = xc2 + R * cos(linspace(delta2 , delta2 - iota2, 100))
                    arc_y2 = yc2 + R * sin(linspace(delta2 , delta2 - iota2, 100))

                else:
                    #If the initial position is inside the second circle
                    delta = arctan2((yc2-y0),(xc2-x0))
                    epsilon1 = delta - theta0
                    omega1 = omega_max if (sin(epsilon1) > 0) else omega_min #CHECK ALL THE CASES
                    v1 = 0
                    t1 = abs(epsilon1/omega1)
                    arc_x1 = x0
                    arc_y1 = y0
                    x1 = x0
                    y1 = y0
                    c1 = sqrt((xc2-x0)**2 - (yc2-y0)**2)
                    x2 = x1 + c1 * cos(alfa)
                    y2 = y1 + c1 * sin(alfa)

                    delta2 = arctan2((yf-y2),(xf-x2))
                    epsilon2 = delta2 - delta 
                    omega3 = omega_max if (sin(epsilon2) > 0) else omega_min #CHECK ALL THE CASES
                    v2 = 0
                    t3 = abs(epsilon2/omega3)
                    arc_x2 = x2
                    arc_y2 = y2
                    x3 = x2
                    y3 = y2
                    c2 = sqrt((xf-xc2)**2 - (yf-yc2)**2)
                    #xf = x1 + c1 * cos(alfa)
                    #y2 = y1 + c1 * sin(alfa)
                     
                t2 = 2*c1/v_max
                #t3 is computed previously
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v1, omega1, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v2, omega3, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

            #CASE 4:Turn right-right
            else:
                xc1 = x0 + R1*cos(theta0 - pi/2)
                yc1 = y0 + R1*sin(theta0-pi/2)
                c1 = sqrt((yc2-yc1)**2 + (xc2-xc1)**2)
                a1 = sqrt(c1**2-R1**2) # TODO: NaN could come from here
                delta1 = arctan2((yc2-yc1),(xc2-xc1)) +2*pi
                beta1 = arcsin(R1/a1)
                x1 = xc1 + R1*cos(delta1+pi/2)
                y1 = yc1 + R1*sin(delta1+pi/2)
                epsilon1 = arctan2((y0-yc1),(x0-xc1)) +2*pi
                x2 = x1 + c1*cos(delta1)
                y2 = y1 + c1*sin(delta1)
                if sqrt((yf-yc2)**2 + (xf-xc2)**2) <= R:
                    c2 = sqrt((yf-y2)**2 + (xf-x2)**2)
                    alfa2 = arctan2((yf-y2),(xf-x2))
                    theta2 = arctan2((y2-y1),(x2-x1))
                    epsilon2 = alfa2 - theta2
                    omega3 = omega_max if (sin(epsilon2) > 0) else omega_min #CHECK ALL THE CASES
                    v2 = 0
                    t3 = abs(epsilon2/omega3)
                    arc_x2 = x2
                    arc_y2 = y2
                    x3 = x2
                    y3 = y2
                else: 
                    delta2 = arctan2((yc2-yf),(xc2-xf))
                    a2 = sqrt((yf-yc2)**2 + (xf-xc2)**2)
                    c2 = sqrt(a2**2 - R**2)
                    beta2 = arcsin(R/a2)
                    x3 = xf + c2*cos(delta2-beta2)
                    y3 = yf + c2*sin(delta2-beta2)
                    epsilon2 = arctan2((y2-yc2),(x2-xc2)) +2*pi
                    zeta2 = arctan2((y3-yc2),(x3-xc2))
                    omega3 = omega_min
                    chord2 = sqrt((x3-x2)**2+(y3-y2)**2)
                    iota2 = 2*arcsin((chord2/2)/R)
                    t3 = R*iota2/v2
                    arc_x2 = xc2 + R * cos(linspace(epsilon2, epsilon2 - iota2, 100))
                    arc_y2 = yc2 + R * sin(linspace(epsilon2, epsilon2 - iota2, 100))

                chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                if R1 > 1e-3:
                    iota1 = 2*arcsin((chord1/2)/R1)
                else:
                    iota1 = abs(theta0 - corridor1.tilt)

                v1 = R1 * abs(omega_min)

                if R1 > 1e-3:
                    t1 = R1 * iota1 /v1
                else: 
                    t1 = iota1 / abs(omega_min)
                t2 = 2*c1/v_max
                #t3 = R*iota2/v2
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v1, omega_min, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v2, omega3, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

                arc_x1 = xc1 + R1 * cos(linspace(epsilon1 , epsilon1 - iota1,100))
                arc_y1 = yc1 + R1 * sin(linspace(epsilon1, epsilon1 - iota1,100))
                #arc_x2 = xc2 + R * cos(linspace(epsilon2, epsilon2 - iota2,100))
                #arc_y2 = yc2 + R * sin(linspace(epsilon2, epsilon2  -iota2 ,100))

        theta1 = arctan2((y2-y1),(x2-x1))
        theta2 = theta1
        theta3 = arctan2((yf-y2),(xf-x2))
        thetaf = theta3
        poses_sequence = np.vstack((np.array(([x0,y0, theta0], [x1, y1, theta1],[x2,y2,theta2], [x3,y3,theta3], [xf,yf,thetaf]))))

        computed_path = np.vstack((
            np.array([arc_x1,arc_y1]).T, 
            [x1,y1], 
            [x2,y2],
            np.array([arc_x2,arc_y2]).T,
            [x3,y3],
            [xf,yf]
        ))

        if plot:
            #Plot solution
            figure_solution = plt.figure()
            angle_complete_circle = linspace(0,2*pi,100)
            ax = figure_solution.add_subplot(111)
            plt.title('x and y position')
            plt.xlabel('x [m]')
            plt.ylabel('y [m]')

            corners = corridor1.corners
            corners.append(corners[0])
            plt.plot([corner[0] for corner in corners], 
                    [corner[1] for corner in corners],
                    'k--', linewidth=1)

            corners = corridor2.corners
            corners.append(corners[0])
            plt.plot([corner[0] for corner in corners], 
                    [corner[1] for corner in corners],
                    'k--', linewidth=1)
            size_dots = 3
            #Plot points
            plt.plot(x0,y0, 'o', color = '#FA8072', label = '(x0,y0)', markersize = size_dots)
            plt.plot(xc1,yc1, 'o', color = '#EE82EE', label = '(xc1, yc1)', markersize = size_dots)
            plt.plot(x1,y1, 'o', color = '#9A0EEA', label = '(x1,y1)', markersize = size_dots)
            plt.plot(x2,y2, 'o', color = 'b', label = '(x2,y2)', markersize = size_dots) #color = '#580F41'
            plt.plot(x3,y3, 'o', color = '#650021', label = '(x3,y3)' ,markersize = size_dots)
            plt.plot(xc2, yc2, 'o', color = 'g', label = '(xc2, yc2)',markersize = size_dots ) #color = '#800000'
            plt.plot(x_corner, y_corner, 'o', color = '#A52A2A', label = '(x_corner, y_corner)',markersize = size_dots )
            plt.plot(xf,yf, 'o', color = '#DC143C', label = '(xf, yf)', markersize = size_dots )

            #Plot solution
            plt.plot(arc_x1, arc_y1, 'b', linewidth=0.8)
            plt.plot(arc_x2, arc_y2, 'b', linewidth=0.8)

            plt.plot([x1, x2], [y1,y2], 'b', linewidth=0.8)
            plt.plot([x3, xf], [y3,yf], 'b', linewidth=0.8)

            plt.plot(xc1+R*cos(angle_complete_circle), yc1 + R*sin(angle_complete_circle), 'k--', linewidth = 0.5)
            plt.plot(xc2+R*cos(angle_complete_circle), yc2+ R*sin(angle_complete_circle), 'k--', linewidth = 0.5)
            
            # plt.axis('square')
            ax.set_aspect('equal')
            plt.legend()

            # control_solution = plt.figure()
            # #ax = figure_solution.add_subplot(111)
            # plt.title('Angular velocity')
            # plt.xlabel('omega [rad/s]')
            # plt.ylabel('time [s]')
            # delta_t1 = np.array([0,t1])
            # delta_t2 = np.array([t1,t1 + t2])
            # delta_t3 = np.array([t1 + t2, t1 + t2 + t3])
            # delta_t4 = np.array([t1 + t2 + t3, t1 + t2 + t3 + t4])

            # plt.plot(delta_t1, [maneuver_sequence[0,1], maneuver_sequence[0,1]])
            # plt.plot(delta_t2, [maneuver_sequence[1,1], maneuver_sequence[1,1]])
            # plt.plot(delta_t3, [maneuver_sequence[2,1], maneuver_sequence[2,1]])
            # plt.plot(delta_t4, [maneuver_sequence[3,1], maneuver_sequence[3,1]])

            # plt.show(block=True)

    return maneuver_sequence, computed_path, poses_sequence


def planner_corridor_sequence(corridor_list, u_bounds, a, b, m, plot, x0, y0, theta0, **kwargs):
    xf = kwargs['xf'] if 'xf' in kwargs else None
    yf = kwargs['yf'] if 'yf' in kwargs else None
    thetaf = kwargs['thetaf'] if 'thetaf' in kwargs else None

    maneuver_list = np.empty((0,3))
    computed_path_list = np.empty((0,2))
    poses_sequence_list = np.empty((0,3))

    for i in range(len(corridor_list)-1):
        
        if i == len(corridor_list)-2:
            maneuver, computed_path, poses_sequence = planner(corridor_list[i], u_bounds, a, b, m, x0, y0, theta0, plot = False, corridor2 = corridor_list[i+1], xf = xf, yf = yf)
            maneuver_list = np.vstack((maneuver_list, maneuver))
            computed_path_list = np.vstack((computed_path_list, computed_path))
            poses_sequence_list = np.vstack((poses_sequence_list, poses_sequence))
            b = 1
        else:
            maneuver, computed_path, poses_sequence = planner(corridor_list[i], u_bounds, a, b, m, x0, y0, theta0, plot = False, corridor2 = corridor_list[i+1])
            maneuver_list = np.vstack((maneuver_list, maneuver[0:2,:]))
            computed_path_list = np.vstack((computed_path_list, computed_path[0:-101]))
            poses_sequence_list = np.vstack((poses_sequence_list, poses_sequence[0:2,:]))
            x0 = poses_sequence[2,0]
            y0 = poses_sequence[2,1]
            theta0 = poses_sequence[2,2] 
            a = 1
    # maneuver, computed_path, poses_sequence = planner(corridor_list[-1], u_bounds, a, b, m, x0, y0, theta0, plot = True)
    # maneuver_list = np.vstack((maneuver_list, maneuver))
    # computed_path_list = np.vstack((computed_path_list, computed_path))

    return maneuver_list, computed_path_list, poses_sequence_list
