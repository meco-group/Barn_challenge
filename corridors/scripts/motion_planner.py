import time
from timeit import default_timer as timer

import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace, arctan2, arcsin

from corridor import *
from corridor_helpers import get_intersection

def compute_goal_point(corridor, m):
    '''
    Compute the goal position given a corridor and a margin m (float). 
    The goal position is computed as the middle point along the forward face of the given corridor, shifted of a quantity equal to m along the 
    line connecting the center and the middle point.

    :param corridor: corridor containing goal position
    :type corridor: corridor
    :param m: margin to avoid that the goal position is on a corridor edge
    :type m: float

    :return: goal_pos = x and y coordinates of the goal position
    :rtype: numpy.ndarray
    '''
    height = corridor.height
    tilt = corridor.tilt - pi/2
    goal_pos = corridor.center + np.array([-(height/2-m)*sin(tilt), (height/2-m)*cos(tilt)])
    return goal_pos

def compute_initial_point(corridor,m):
    '''
    Compute the initial position given a corridor and a margin m (float). 
    The initial position is computed as the middle point along the backward face of the given corridor, shifted of a quantity equal to m along the 
    line connecting the center and the middle point.

    :param corridor: corridor containing initial position
    :type corridor: corridor
    :param m: margin to avoid that the initial position is on a corridor edge
    :type m: float

    :return: initial_pos = x and y coordinates of the initial position
    :rtype: numpy.ndarray
    '''
    height = corridor.height
    tilt = corridor.tilt - pi/2
    initial_pos = corridor.center + np.array([(height/2-m)*sin(tilt), -(height/2-m)*cos(tilt)])
    return initial_pos

def check_inside_one_point(corridor, point):
        '''Check if one point is inside the corridor. This function is based on check_inside

        :param point: array with x and y coordinates of the point to be checked
            whether it is inside the corridor
        :type point: numpy.ndarray
        :param corridor: corridor to be checked whether it contains the point
        :type corridor: corridor

        :return: bool with True if point is inside the corridor
        :rtype: bool
        '''
        Wtransp = corridor.W.T
        hompoint = np.append(point,1)
        # Check if point is inside the corridor
        if np.all(Wtransp @ hompoint <= 1e-3):
            return True
        return False

def get_corner_point(parent, child):
    '''
    Given a parent and a child corridor, compute the corner point which corresponds to the intersection of their edges.

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
    corner_point =np.empty([0,2]) #initialize a np.array containing the corner points candidates
    turn_left = False

    #select the correct faces to check for the corner point, depending whether the maneuver is turning right or left
    if tilt2 >= tilt1: #turn left
        turn_left = True
        select_edges1 = np.array([0,2,3]) #check forward, backward and left face of child 
        select_edges2 = np.array([0,3]) #check forward and left faces of parent
    else: #turn right
        select_edges1 = np.array([0,2,1]) #check forward, right and backward face of child
        select_edges2 = np.array([0,1]) #check forward and right faces of parent

    for i in select_edges1:
        for k in select_edges2:
            w1 = W_child[:, i]
            w2 = W_parent[:, k]
            intersection_point = get_intersection(w1, w2)
            if check_inside_one_point(parent, intersection_point) and check_inside_one_point(child, intersection_point):
                #if turn left and there are more than one candidate points, take the one with lower x coordinate (more on the left)
                if turn_left and np.all(intersection_point[0]< corner_point[:,0]):
                    corner_point = np.vstack((intersection_point,corner_point))
                #if turn right and there are more than one candidate points, take the one with larger x coordinate (more on the right)
                elif not(turn_left) and np.all(intersection_point[0]> corner_point[:,0]):
                    corner_point = np.vstack((intersection_point, corner_point))
                else:
                    corner_point = np.vstack((corner_point,intersection_point))

    #you could also just store all the points in corner_point and at the end get the one with minimum/maximum x coordinate whether you are turning left or right
    x_corner = corner_point[0,0]
    y_corner = corner_point[0,1]
    return x_corner, y_corner


def plot_trajectory(corridor1, R, x0, y0, xf, yf, x1, y1, x_center1, y_center1, arc_x1, arc_y1, angle1, **kwargs):
    corridor2 = kwargs['corridor2'] if 'corridor2' in kwargs else None

    if corridor2 == None:
        #Plot solution
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
        #Plot triangle rectangle, just to be sure
        plt.plot([x1,xf],[y1,yf],'b-', linewidth=0.8)
        plt.plot(x0,y0, 'ro')
        plt.plot(x1,y1, 'ro')
        plt.plot(xf, yf, 'ro')
        plt.plot(arc_x1, arc_y1, 'k')
        plt.plot([x_center1 + R * cos(angle1)], [y_center1+R*sin(angle1)], 'r')

        # plt.axis('square')
        ax.set_aspect('equal')
        # plt.legend()
        plt.show(block=True)


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
        #initialize the output with two maneuvers
        maneuver_sequence = np.empty((2,3))
        goal_pos = compute_goal_point(corridor1,m) #not best implemention, you compute goal_pos but you might not need it 
        xf = kwargs['xf'] if 'xf' in kwargs else goal_pos[0]
        yf = kwargs['yf'] if 'yf' in kwargs else goal_pos[1]

        
        #Compute the angle with respect to the horizontal of line connecting initial and goal pos
        ref_orientation = arctan2(yf-y0,xf-x0)

        if ref_orientation >= theta0:

            #Compute center point of osculating circle
            x_center1 = x0 - R * sin(theta0)
            y_center1 = y0 + R * cos(theta0)

            #Build triangle rectangle between (x_center, y_center), (xf,yf), (x1,y1)
            a1 = sqrt((xf-x_center1)**2 + (yf-y_center1)**2)
            c1 = sqrt(a1**2 - R**2) #lenght line segment
            alfa = arctan2((yf-y_center1),(xf-x_center1))
            gamma = arcsin(c1/a1)
            zeta = alfa - gamma
            x1 = x_center1 + R*cos(zeta)
            y1 = y_center1 + R*sin(zeta)
            eta = arctan2((y0 - y_center1), (x0 - x_center1))
            epsilon = abs(zeta) +abs(eta) #arclength
            angle1 = linspace(eta, eta + epsilon, 100, endpoint = True)
            arc_x1 = x_center1+R*cos(angle1)
            arc_y1 = y_center1+R*sin(angle1)

            t1 = R * epsilon/ v_max
            t2 = c1 / v_max

            if arctan2((yf-y1),(xf-x1)) >= theta0:
                omega = omega_max
            else:
                omega = omega_min

            maneuver_sequence[0,:] = np.array([v_max, omega, t1])
            maneuver_sequence[1,:] = np.array([v_max, 0, t2])

        else:
            #Compute center point of osculating circle
            x_center1 = x0 + R * cos(theta0-pi/2)
            y_center1 = y0 + R * sin(theta0-pi/2)

            #Build triangle rectangle between (x_center, y_center), (xf,yf), (x1,y1)
            a1 = sqrt((xf-x_center1)**2 + (yf-y_center1)**2)
            c1 = sqrt(a1**2 - R**2) #lenght line segment
            alfa = arctan2((yf-y_center1),(xf-x_center1))
            gamma = arcsin(c1/a1)
            zeta = alfa + gamma
            x1 = x_center1 + R*cos(zeta)
            y1 = y_center1 + R*sin(zeta)
            eta = arctan2((y0 - y_center1), (x0 - x_center1))
            epsilon = pi - abs(eta) + pi - abs(eta) #arclength
            angle1 = np.hstack((linspace(eta, -pi, 100, endpoint = True), linspace(zeta, pi, 100, endpoint = True)))
            arc_x1 = x_center1+R*cos(angle1)
            arc_y1 = y_center1+R*sin(angle1)

            t1 = R * epsilon/ v_max
            t2 = c1 / v_max

            if arctan2((yf-y1),(xf-x1)) >= theta0:
                omega = omega_max
            else:
                omega = omega_min

            maneuver_sequence[0,:] = np.array([v_max, omega, t1])
            maneuver_sequence[1,:] = np.array([v_max, 0, t2])

        computed_path = np.vstack((
            np.array([arc_x1,arc_y1]).T, 
            [xf,yf]
        ))

        if plot:
            plot_trajectory(corridor1, R, x0, y0, xf, yf, x1, y1, x_center1, y_center1, arc_x1, arc_y1, angle1)

    else:
        goal_pos = compute_goal_point(corridor2,m)
        xf = kwargs['xf'] if 'xf' in kwargs else goal_pos[0]
        yf = kwargs['yf'] if 'yf' in kwargs else goal_pos[1]
        
        tilt1 = corridor1.tilt-pi/2
        tilt2 = corridor2.tilt-pi/2

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
            tilt1 = corridor1.tilt-pi/2
            tilt2 = corridor2.tilt-pi/2
            #Compute the coordinates of the center of circle 2
            #angle = (pi - arccos(cos(tilt1)*cos(tilt2)+sin(tilt1)*sin(tilt2)))/2
            angle = (pi - (tilt2-tilt1))/2
            xc2 = x_corner + (R-a/2-m)*cos(pi/2 + tilt2 + angle)
            yc2 = y_corner + (R-a/2-m)*sin(pi/2 + tilt2 + angle)

            if theta0 <= pi/2 or theta0 >= 3*pi/2:

                #Compute the coordinates of the center of circle 1
                xc1 = x0 + R * cos(theta0 + pi/2)
                yc1 = y0 + R * sin(theta0 + pi/2)

                c1 = sqrt((yc2 - yc1)**2 + (xc2 - xc1)**2)
                delta1 = arctan2((yc2 - yc1), (xc2-xc1))
                epsilon1 = delta1 - pi/2
                zeta1 = arctan2((y0 - yc1),(x0 - xc1))
                eta1 = abs(epsilon1) + abs(zeta1)
                x1 = xc1 + R * cos(epsilon1)
                y1 = yc1 + R * sin(epsilon1)
                x2 = x1 + c1 * cos(epsilon1 + pi/2)
                y2 = y1 + c1 * sin(epsilon1 + pi/2)

                epsilon2 = arctan2((y2 - yc2), (x2 - xc2))
                delta2 = arctan2((yc2-yf),(xc2-xf))
                a2 = sqrt((yf-yc2)**2+(xf-xc2)**2)
                c2 = sqrt(a2**2 - R**2)
                beta2 = arcsin(R/a2)
                iota2 = delta2 + beta2
                x3 = xf + c2 * cos(iota2)
                y3 = yf + c2 * sin(iota2)
                zeta2 = arctan2((y3-yc2),(x3-xc2))
                eta2 = zeta2 - epsilon2

                t1 = R * eta1 /v_max
                t2 = c1/v_max
                t3 = R*eta2/v_max
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v_max, omega_max, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v_max, omega_max, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

                arc_x1 = xc1 + R * cos(linspace(zeta1, epsilon1, 100))
                arc_y1 = yc1 + R * sin(linspace(zeta1, epsilon1, 100))
                arc_x2 = xc2 + R * cos(linspace(epsilon2, zeta2, 100))
                arc_y2 = yc2 + R * sin(linspace(epsilon2, zeta2, 100))
            
            else:
                xc1 = x0 + R*cos(theta0 - pi/2)
                yc1 = y0 + R*sin(theta0 - pi/2)
                a1 = sqrt((yc2-yc1)**2+(xc2-xc1)**2)
                c1 = sqrt(a1**2-R**2)
                beta1 = arcsin(R/a1)
                delta1 = arctan2((yc2-yc1),(xc2-xc1))
                epsilon1 = delta1 - beta1
                x2 = xc1 + c1*cos(epsilon1)
                y2 = yc1 + c1*sin(epsilon1)
                gamma1 = arcsin(R/c1)
                eta1 = pi/2 - gamma1
                x1 = xc1 + R * cos(epsilon1 + eta1)
                y1 = yc1 + R *sin(epsilon1+eta1)
                dist = sqrt((y2-y1)**2 + (x2-x1)**2)
                chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                zeta1 = 2*arcsin((chord1/2)/R)
                iota1 = arctan2((y0-yc1),(x0-xc1)) + 2*pi

                epsilon2 = arctan2((y2-yc2),(x2-xc2))
                delta2 = arctan2((yc2-yf),(xc2-xf))
                a2 = sqrt((yf-yc2)**2+(xf-xc2)**2)
                c2 = sqrt(a2**2 - R**2)
                beta2 = arcsin(R/a2)
                iota2 = delta2 + beta2
                x3 = xf + c2*cos(iota2)
                y3 = yf + c2*sin(iota2)
                zeta2 = arctan2((y3-yc2),(x3-xc2))
                eta2 = zeta2 - epsilon2

                t1 = R * zeta1 /v_max
                t2 = dist/v_max
                t3 = R*eta2/v_max
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v_max, omega_min, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v_max, omega_max, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

                arc_x1 = xc1 + R * cos(linspace(iota1, iota1 - zeta1, 100))
                arc_y1 = yc1 + R * sin(linspace(iota1, iota1 - zeta1, 100))
                arc_x2 = xc2 + R * cos(linspace(epsilon2, zeta2, 100))
                arc_y2 = yc2 + R * sin(linspace(epsilon2, zeta2, 100))

            angle_complete_circle = linspace(0,2*pi,100)

        elif turn_right:
            maneuver_sequence = np.empty((4,3))
            #Compute the center coordinates
            x_corner, y_corner = get_corner_point(corridor1, corridor2)
            tilt1 = corridor1.tilt-pi/2
            tilt2 = corridor2.tilt-pi/2
            #Compute the coordinates of the center of circle 2
            #angle = (pi - arccos(cos(tilt1)*cos(tilt2)+sin(tilt1)*sin(tilt2)))/2
            angle = (pi - abs(tilt2-tilt1))/2
            xc2 = x_corner + (R-a/2-m)*cos(pi/2 + tilt2 - angle)
            yc2 = y_corner + (R-a/2-m)*sin(pi/2 + tilt2 - angle)

            if theta0 <= pi/2 or theta0 >= 3*pi/2:
                xc1 = x0 + cos(theta0 + pi/2)
                yc1 = y0 + sin(theta0 + pi/2)
                a1 = sqrt((yc2-yc1)**2 + (xc2-xc1)**2)/2
                c1 = sqrt(a1**2 - R**2)
                delta1 = arctan2((y0-yc1),(x0-xc1))
                epsilon1 = arctan2((yc2-yc1),(xc2-xc1))
                gamma1 = arcsin(c1/a1)

                eta1 = epsilon1 - gamma1
                x1 = xc1 + R*cos(eta1)
                y1 = yc1 + R*sin(eta1)
                x2 = x1+2*c1*cos(eta1 + pi/2)
                y2 = y1 + 2*c1*sin(eta1+pi/2)
                delta2 = arctan2((y2-yc2),(x2-xc2))
                epsilon2 = arctan2((yc2-yf),(xc2-xf))
                a2 = sqrt((yf-yc2)**2+(xf-xc2)**2)
                c2 = sqrt(a2**2-R**2)
                beta2 = arcsin(R/a2)
                x3 = xf + c2*cos(epsilon2 - beta2)
                y3 = yf + c2*sin(epsilon2-beta2)
                eta2 = arctan2((y3-yc2),(x3-xc2))
                
                chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                iota1 = 2*arcsin((chord1/2)/R)
                chord2 = sqrt((x3-x2)**2+(y3-y2)**2)
                iota2 = 2*arcsin((chord2/2)/R)

                t1 = R * iota1 /v_max
                t2 = 2*c1/v_max
                t3 = R*iota2/v_max
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v_max, omega_max, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v_max, omega_min, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

                arc_x1 = xc1 + R * cos(linspace(delta1, eta1,100))
                arc_y1 = yc1 + R * sin(linspace(delta1, eta1,100))
                arc_x2 = xc2 + R * cos(linspace(delta2 + 2*pi, eta2,100))
                arc_y2 = yc2 + R * sin(linspace(delta2 +2*pi, eta2,100))

            else:
                xc1 = x0 + R*cos(theta0 - pi/2)
                yc1 = y0 + R*sin(theta0-pi/2)
                c1 = sqrt((yc2-yc1)**2 + (xc2-xc1)**2)
                a1 = sqrt(c1**2-R**2)
                delta1 = arctan2((yc2-yc1),(xc2-xc1))
                beta1 = arcsin(R/a1)
                x1 = xc1 + R*cos(delta1+pi/2)
                y1 = yc1 + R*sin(delta1+pi/2)
                epsilon1 = arctan2((y0-yc1),(x0-xc1))
                x2 = x1 + c1*cos(delta1)
                y2 = y1 + c1*sin(delta1)
                delta2 = arctan2((yc2-yf),(xc2-xf))
                a2 = sqrt((yf-yc2)**2 + (xf-xc2)**2)
                c2 = sqrt(a2**2 - R**2)
                beta2 = arcsin(R/a2)
                x3 = xf + c2*cos(delta2-beta2)
                y3 = yf + c2*sin(delta2-beta2)
                epsilon2 = arctan2((y2-yc2),(x2-xc2))
                zeta2 = arctan2((y3-yc2),(x3-xc2))

                chord1 = sqrt((x1-x0)**2+(y1-y0)**2)
                iota1 = 2*arcsin((chord1/2)/R)
                chord2 = sqrt((x3-x2)**2+(y3-y2)**2)
                iota2 = 2*arcsin((chord2/2)/R)
                t1 = R * iota1 /v_max
                t2 = 2*c1/v_max
                t3 = R*iota2/v_max
                t4 = c2/v_max

                maneuver_sequence[0,:] = np.array([v_max, omega_min, t1])
                maneuver_sequence[1,:] = np.array([v_max, 0, t2])
                maneuver_sequence[2,:] = np.array([v_max, omega_min, t3])
                maneuver_sequence[3,:] = np.array([v_max, 0, t4])

                arc_x1 = xc1 + R * cos(linspace(epsilon1 + 2*pi, delta1 + pi/2,100))
                arc_y1 = yc1 + R * sin(linspace(epsilon1 + 2*pi, delta1 + pi/2,100))
                arc_x2 = xc2 + R * cos(linspace(epsilon2 + 2*pi, epsilon2 + 2*pi - iota2,100))
                arc_y2 = yc2 + R * sin(linspace(epsilon2 + 2*pi, epsilon2 + 2*pi - iota2 ,100))

        
        
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
            # plt.legend()

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

            plt.show(block=True)

    return maneuver_sequence, computed_path
