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

def detect_obstacle(pnts2D, distance):
    robot_width = 0.4
    closest_points = []
    
    for p in pnts2D:
        if (p[0] < robot_width/2 + 0.2 and p[0] > - robot_width/2 - 0.2) and p[1] < distance:
            closest_points.append(p[1])
        
    if len(closest_points) >= 1:
        front_limit = min(closest_points)
        pnts2Dpom = pnts2D
        # pnts2Dpom = []
        # for p in pnts2D:
        #     if p[1] < front_limit + 0.2:
        #         pnts2Dpom.append(p)
        left_limit_point = min(pnts2Dpom, key=lambda pnt: pnt[0])
        right_limit_point = max(pnts2Dpom, key=lambda pnt: pnt[0]) 
        return front_limit, left_limit_point, right_limit_point
    else:
        return None, None, None

def avoid_obstacle(pnts2D, t_delay, distance):
    robot_width = 0.4
    robot_velocity = 0.15
    camera_hfov = 1.047*0.85
    closest_points_z = []
    left_side_points = []
    right_side_points = []
    for p in pnts2D:
        if (p[0] < robot_width/2 + 0.2 and p[0] > - robot_width/2 - 0.2) and p[1] < distance:
            closest_points_z.append(p[1])
        if (p[0] < 0):
            left_side_points.append(p)
        elif(p[0] > 0):
            right_side_points.append(p)

    if len(closest_points_z) >= 1:
        front_limit = min(closest_points_z) - t_delay*robot_velocity
        pnts2Dpom = pnts2D
        # pnts2Dpom = []
        # for p in pnts2D:
        #     if p[1] < front_limit + 0.2:
        #         pnts2Dpom.append(p)
        left_limit_point = min(pnts2Dpom, key=lambda pnt: pnt[0])
        right_limit_point = max(pnts2Dpom, key=lambda pnt: pnt[0])  
        left_limit_z = left_limit_point[1] - t_delay*robot_velocity
        right_limit_z = right_limit_point[1] - t_delay*robot_velocity
        left_limit_x = left_limit_point[0]
        right_limit_x = right_limit_point[0]
        left_side_fov_occupied = False
        right_side_fov_occupied = False

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
        
        if (front_limit < 0.5):
            return 180, 0
        elif (right_limit_x == left_limit_x):
            direction = direction=np.sign(left_limit_x)
            side_limit = right_limit_x
        elif ((left_side_fov_occupied and (not right_side_fov_occupied or left_side_min_depth <= right_side_min_depth)) or
            (not left_side_fov_occupied and not right_side_fov_occupied and abs(left_limit_x) > abs(right_limit_x))):
            direction = -1
            side_limit = right_limit_x
            front_limit = min(front_limit, right_limit_z)
        else:
            direction = 1
            side_limit = left_limit_x
            front_limit = min(front_limit, left_limit_z)
        angle, x = get_turning_angle(robot_width, front_limit, side_limit, direction)     
        return angle, x 
    else:
        return None, None

# def avoid_obstacle_modified(pnts2D, t_delay):
#     robot_width = 0.4
#     robot_velocity = 0.15
#     camera_hfov = 1.047*0.85
#     closest_points_z = []
#     left_side_points = []
#     right_side_points = []
#     for p in pnts2D:
#         if (p[0] < robot_width/2 + 0.2 and p[0] > - robot_width/2 - 0.2) and p[1] < 1:
#             closest_points_z.append(p[1])

#     if len(closest_points_z) >= 1:
#         front_limit = min(closest_points_z) - t_delay*robot_velocity


def get_point_to_line_distance(P0, P1_line, P2_line):
    x1 = P1_line[0]
    y1 = P1_line[1]
    x2 = P2_line[0]
    y2 = P2_line[1]
    x0 = P0[0]
    y0 = P0[1]
    d = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / np.sqrt(np.square(x2-x1) + np.square(y2-y1))
    return d

def get_turning_angle(robot_width, front_limit, side_limit, direction):
    x = side_limit
    P0 = [side_limit, front_limit]
    P1_line= [0, 0]
    while get_point_to_line_distance(P0, P1_line, P2_line=[x, front_limit]) < (robot_width/2 + 0.2):
        x = x + 0.1*(-direction)
    angle = math.degrees(np.arcsin(abs(x)/np.linalg.norm([x, front_limit])))*direction
    return angle, x
    

def transform_to_2D_XZ_coordinate_system(pnts3D):
    pnts2D_new = (np.delete(pnts3D, 1, 1))
    return pnts2D_new    
    
