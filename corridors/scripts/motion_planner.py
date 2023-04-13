from rockit import *
from casadi import *
import time
from timeit import default_timer as timer

import matplotlib.pyplot as plt
from numpy import pi, cos, sin, tan, sqrt, linspace

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
    tilt = corridor.tilt
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
    tilt = corridor.tilt
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


#Compute analytical solution
# compute_trajectory(corridor1, corridor2, u_bounds, x0, y0, xf, yf, a, b, m)

def compute_trajectory(corridor1, u_bounds, a, b, m, x0, y0, theta0, **kwargs):
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

    :optional param corridor2: arrival corridor. This parameter is provided in case a second corridor is available
    :type corridor2: corridor
    :optional param xf: x coordinate of goal position
    :type xf: float
    :optional param yf: y coordinate of goal position
    :type yf: float

    :return: maneuver contains the control inputs and time maneuver = [v, omega, time]
    :rtype: np.array
    '''
    #Initialize output
    maneuver = np.empty((0,3))
    #Unpack variables
    v_min = u_bounds[0]
    v_max = u_bounds[1]
    omega_min = u_bounds[2]
    omega_max = u_bounds[3]
    corridor2 = kwargs['corridor2'] if 'corridor2' in kwargs else None

    ## Compute the trajetory in case you have only one corridor
    if corridor2 == None:  
        goal_pos = compute_goal_point(corridor1,m) #not best implemention, you compute goal_pos but you might not need it 
        xf = kwargs['xf'] if 'xf' in kwargs else goal_pos[0]
        yf = kwargs['yf'] if 'yf' in kwargs else goal_pos[1]

        #Compute distance between the initial and goal position
        c = sqrt((xf-x0)**2+(yf-y0)**2)
        #Compute the angle with respect to the horizontal of line connecting initial and goal pos
        gamma = atan2(yf-y0,xf-x0)
        #Compute the time needed to arrive to goal position in minimum time
        t1 = v_max * c
        maneuver = np.array([v_max, 0, t1])

        if theta0 < gamma:
            t_align = (gamma - theta0)/omega_max
            maneuver = np.vstack((np.array([0,omega_max,t_align]), maneuver))
        elif theta0 > gamma:
            t_align = (theta0-gamma)/omega_min
            maneuver = np.vstack((np.array([0,omega_min,t_align]), maneuver))
        else:
            t_align = 0

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

        text = f'Analytical solution \nTotal time = {"{:.8f}".format(t_align + t1)}s'
        plt.plot([x0,xf],[y0,yf],'b-', linewidth=0.8, label=text)

        plt.axis('square')
        plt.legend()
        plt.show(block=True)

    #In case you have two corridors available, compute the trajectory whithin the two of them.
    else:
        goal_pos = compute_goal_point(corridor2,m)
        xf = kwargs['xf'] if 'xf' in kwargs else goal_pos[0]
        yf = kwargs['yf'] if 'yf' in kwargs else goal_pos[1]
        
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
        
        #Analytical solution
        #Compute the radius
        R = abs(v_max/omega_max) 
        circle_center_angle = abs(tilt1 - tilt2)/2
        #Compute the coordinates of the circle center considering the footprint 
        if turn_left:
            #Coordinates of the corner point
            x_corner, y_corner = get_corner_point(corridor1, corridor2)

            #Coordinates of the center of the osculating circle
            x_center = x_corner - (R-a/2-m)*cos(circle_center_angle)
            y_center = y_corner - (R-a/2-m)*sin(circle_center_angle)
            #Compute (x1,y1) 
            a1 = sqrt((x_center-x0)**2+(y_center-y0)**2)
            c1 = sqrt(a1**2-R**2)
            beta1 = arcsin(R/a1)
            alfa1 = arctan2((y_center-y0),(x_center-x0))
            gamma1 = alfa1 - beta1

            x1 = x0 + c1*cos(gamma1)
            y1 = y0+ c1*sin(gamma1)

            #Compute (x2,y2)
            a2 = sqrt((xf-x_center)**2+(yf-y_center)**2)
            c2 = sqrt(a2**2-R**2)
            beta2 = arcsin(R/a2)
            alfa2 = arctan2((xf-x_center),(yf-y_center))
            gamma2 = alfa2 - beta2

            x2 = xf-c2*sin(gamma2)
            y2 = yf-c2*cos(gamma2)

            #Compute the curvilinear arc segment
            chord = sqrt((x2-x1)**2 + (y2-y1)**2)
            theta = 2*arcsin((chord/2)/R)
            epsilon = arctan((y_center-y2)/(x_center-x2))
            angle_arc_segment = linspace(epsilon,epsilon-theta,100, endpoint = True)
            arc_x = x_center+R*cos(angle_arc_segment)
            arc_y = y_center+R*sin(angle_arc_segment)
            angle_complete_circle = linspace(0,2*pi,100)

            #Compute control inputs
            t1 = v_max * c1
            t2 = v_max * R * theta
            t3 = v_max * c2  
            maneuver = np.array([[v_max, 0 , t1],[v_max, omega_max , t2], [v_max, 0 , t3]])

            if theta0 < gamma1:
                t_align = (gamma1 - theta0)/ omega_max
                maneuver = np.vstack((np.array([0,omega_max,t_align]), maneuver))
            elif theta0 > gamma1:
                t_align = abs((theta0-gamma1)/omega_min)
                maneuver = np.vstack((np.array([0,omega_min,t_align]), maneuver))
            else:
                t_align = 0

        elif turn_right:
        #Coordinates of the corner point
            x_corner, y_corner =get_corner_point(corridor1, corridor2)

            #Coordinates of the center of the osculating circle
            x_center = x_corner + (R-a/2-m)*cos(circle_center_angle)
            y_center = y_corner - (R-a/2-m)*sin(circle_center_angle)
            #Compute (x1,y1) 
            a1 = sqrt((x_center-x0)**2+(y_center-y0)**2)
            c1 = sqrt(a1**2-R**2)
            beta1 = arcsin(R/a1)
            alfa1 = arctan2((y_center-y0),(x_center-x0))
            gamma1 = alfa1 + beta1

            x1 = x0+ c1*cos(gamma1)
            y1 = y0+ c1*sin(gamma1)

            #Compute (x2,y2)
            a2 = sqrt((xf-x_center)**2+(yf-y_center)**2)
            c2 = sqrt(a2**2-R**2)
            beta2 = arcsin(R/a2)
            alfa2 = arctan2((y_center-yf), (x_center-xf))
            # gamma2 =abs(alfa2 - beta2) - pi/2

            # x2 = xf-c2*sin(gamma2)
            # y2 = yf-c2*cos(gamma2)
            gamma2 = alfa2 = beta2
            x2 = xf + c2*cos(gamma2)
            y2 = yf + c2*sin(gamma2)

            #Compute the curvilinear arc segment
            chord = sqrt((x2-x1)**2 + (y2-y1)**2)
            theta = 2*arcsin((chord/2)/R)
            epsilon = abs(arctan((y_center-y2)/(x_center-x2)))
            epsilon2 = abs(arctan((y_center-y1)/(x_center-x1)))
            angle_arc_segment = linspace(epsilon2, epsilon, 100, endpoint = True)
            arc_x = x_center-R*cos(angle_arc_segment)
            arc_y = y_center+R*sin(angle_arc_segment)
            angle_complete_circle = linspace(0,2*pi,100)

            if theta0 < gamma1:
                t_align = (gamma1 - theta0)/ omega_max
                maneuver = np.array(0,omega_max,t_align)
            elif theta0 > gamma1:
                t_align = abs((theta0-gamma1)/omega_min)
                maneuver = np.array(0,omega_max,t_align)
            else:
                t_align = 0

        elif go_straight:
            #Compute distance between the initial and goal position
            c = sqrt((xf-x0)**2+(yf-y0)**2)
            gamma = atan2(yf-y0,xf-x0)

            if theta0 < gamma:
                t_align = (gamma - theta0)/omega_max
                maneuver = np.array(0,omega_max,t_align)
            elif theta0 > gamma:
                t_align = (theta0-gamma)/omega_min
                maneuver = np.array(0,omega_min,t_align)
            else:
                t_align = 0
            #Compute the time needed to arrive to goal position in minimum time
            t1 = v_max * c
            maneuver = np.array([v_max, 0, t1])
           
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

            #Plot solution
            text = f'Analytical solution \nTotal time = {"{:.8f}".format(t1)}s'
            plt.plot([x0,xf],[y0,yf],'b-', linewidth=0.8, label=text)

            plt.axis('square')
            plt.legend()
            plt.show(block=True)

        #Compute the time instants in which to start and stop steering, plus final time
        #Formula = v_max * length 
        if turn_right or turn_left: 

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

            corners = corridor2.corners
            corners.append(corners[0])
            plt.plot([corner[0] for corner in corners], 
                    [corner[1] for corner in corners],
                    'k--', linewidth=1)
                    #Plot solution
            text = f'Analytical solution \nTotal time = {"{:.8f}".format(t3)}s \nt1 = {"{:.3f}".format(t1)}s \nt2 = {"{:.2f}".format(t2)}s'
            plt.plot(arc_x, arc_y, 'b', linewidth=0.8, label=text)
            plt.plot([x1, x0], [y1,y0], 'b', linewidth=0.8)
            plt.plot([x2, xf], [y2,yf], 'b', linewidth=0.8)
            plt.plot([x0,xf],[y0,yf],'ro')
            plt.plot([x_center],[y_center],'go')
            plt.plot([x_corner],[y_corner],'ro')
            plt.plot(x_center+R*cos(angle_complete_circle), y_center + R*sin(angle_complete_circle), 'k--')
            plt.axis('square')
            plt.legend()

            plt.show(block=True)
        return maneuver





