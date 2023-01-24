#!/usr/bin/env python

import numpy as np
import math

def find_average_depth(pnts3D):
    Z = [z[2] for z in pnts3D]
    if len(Z) > 0:
        average_depth = sum(Z)/len(Z)
    else:
        average_depth = None
    return average_depth

def avoid_obstacle(pnts2D, t_delay, distance, robot_velocity, pos_new):
    robot_width = 0.4
    camera_hfov = 1.047*0.85
    left_side_points = []
    right_side_points = []
    points_in_the_way = []
    p_R_right_tangent = []
    p_R_left_tangent = []
    p_L_right_tangent = []
    p_L_left_tangent = []
    k_R_min = 10000
    k_R_max = 0
    k_L_min = 0
    k_L_max = -10000

    for p in pnts2D:
        p_new = [p[0], p[1]-t_delay*robot_velocity]
        if (p[0] < robot_width/2 + 0.15 and p[0] > - robot_width/2 - 0.15) and p_new[1] < distance:
            points_in_the_way.append(p_new)
        if (p_new[0] < 0):
            p_L = p_new
            k = p_L[1]/p_L[0]
            if k > k_L_max:
                k_L_max = k
                p_L_left_tangent = p_L
            if k < k_L_min:
                k_L_min = k
                p_L_right_tangent = p_L  
            left_side_points.append(p_L)
        elif(p_new[0] >= 0):
            p_R = p_new
            if p_R[0] == 0:
                k = 10000
            else:
                k = p_R[1]/p_R[0]
            if k < k_R_min:
                k_R_min = k
                p_R_right_tangent = p_R
            if k > k_R_max:
                k_R_max = k
                p_R_left_tangent = p_R
            right_side_points.append(p_R)
    
    if len(points_in_the_way) > 0 and len(pnts2D) > 3:
        front_limit_point = min(points_in_the_way, key=lambda pnt: pnt[1])
        left_limit_point = min(pnts2D, key=lambda pnt: pnt[0])
        right_limit_point = max(pnts2D, key=lambda pnt: pnt[0])  
        left_limit_x = left_limit_point[0]
        right_limit_x = right_limit_point[0]
        left_side_fov_occupied = False
        right_side_fov_occupied = False
        tangent_point = front_limit_point
        z_real = abs(float(pos_new[0])) - 0.189972
        z_avg = np.sum(pnts2D, 0)[1]/len(pnts2D)
        z_err = (abs(z_avg - z_real)/z_real)*100
        x_min_err = (abs(left_limit_x + 0.5)/0.5)*100
        x_max_err = (abs(right_limit_x - 0.5)/0.5)*100

        if len(left_side_points) > 0:
            for p in left_side_points:
                x_min = - math.tan(camera_hfov/2)*p[1]
                if p[0] <= x_min:
                    left_side_fov_occupied = True
                    left_side_min_depth = min(left_side_points, key=lambda pnt: pnt[1])[1]
                    break
        if len(right_side_points) > 0:        
            for p in right_side_points:
                x_max = math.tan(camera_hfov/2)*p[1]
                if p[0] >= x_max:
                    right_side_fov_occupied = True
                    right_side_min_depth = min(right_side_points, key=lambda pnt: pnt[1])[1]
                    break
        
        if (front_limit_point[1] < 0.5):
            return 180, 0, tangent_point
        elif (right_limit_x == left_limit_x):
            direction = direction=np.sign(left_limit_x)
        elif ((left_side_fov_occupied and (not right_side_fov_occupied or left_side_min_depth <= right_side_min_depth)) or
            (not left_side_fov_occupied and not right_side_fov_occupied and abs(left_limit_x) > abs(right_limit_x))):
            direction = -1
            if len(right_side_points) == 0:
                tangent_point = p_L_right_tangent
            else:
                tangent_point = p_R_right_tangent
        else:
            direction = 1
            if len(left_side_points) == 0:
                tangent_point = p_R_left_tangent
            else:
                tangent_point = p_L_left_tangent
        angle, x = get_turning_angle(robot_width, tangent_point, direction)     
        return angle, x, tangent_point
    else:
        return None, None, None

def get_point_to_line_distance(P0, P1_line, P2_line):
    x1 = P1_line[0]
    y1 = P1_line[1]
    x2 = P2_line[0]
    y2 = P2_line[1]
    x0 = P0[0]
    y0 = P0[1]
    d = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / np.sqrt(np.square(x2-x1) + np.square(y2-y1))
    return d

def get_turning_angle(robot_width, tangent_point, direction):
    x = tangent_point[0]
    P0 = tangent_point
    P1_line= [0, 0]
    while get_point_to_line_distance(P0, P1_line, P2_line=[x, tangent_point[1]]) < (robot_width/2 + 0.2):
        x = x + 0.1*(-direction)
    angle = math.degrees(np.arcsin(abs(x)/np.linalg.norm([x, tangent_point[1]])))*direction
    return angle, x

def transform_to_2D_XZ_coordinate_system(pnts3D):
    pnts2D_new = (np.delete(pnts3D, 1, 1))
    return pnts2D_new    
    
