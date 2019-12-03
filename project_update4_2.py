#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 17:47:25 2019

@author: yihuicui
"""

import cv2
import numpy as np
import sys
import heapq
import matplotlib.pyplot as plt
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
from IPython.display import clear_output
from IPython.display import display
import ctypes
import collections
from PID import PID


path = r"C:\Users\alber\Documents\ECE_470\ECE470_Project\top_view.png"
img = cv2.imread(path)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#img = cv2.resize(img, (500, 500))
#img = cv2.flip( img, 0 )

plt.imshow(img)
plt.show()
def obstacles_grid(img):
    # getting the walls 
    mask_wall = cv2.inRange(img, np.array([230,230,230]),np.array([235,235,235]))
    # getting the rims
    mask_rim = cv2.inRange(img, 0, 0)
    mask_total = cv2.bitwise_or(mask_wall,mask_rim,mask_rim)
    mask_total = cv2.bitwise_not(mask_total)
    return mask_total
img_obs = obstacles_grid(img)
plt.imshow(img_obs, cmap="gray")
plt.show()


def get_front_disk_position (img):
    front_disk_mask = cv2.inRange(img, np.array([100,140,205]), np.array([105,145,210]))
    contours, hierarchy = cv2.findContours(front_disk_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #no need for the for loop but it is used to avoid errors
    for c in contours:
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return np.array([cY,cX])

def get_back_disk_position (img):
    rear_disk_mask = cv2.inRange(img, np.array([75,185,185]), np.array([80,190,190]))    
    contours, hierarchy = cv2.findContours(rear_disk_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #no need for the for loop but it is used to avoid errors
    for c in contours:
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return np.array([cY,cX])
def get_robot_orientation(img):
    front_disk_position = get_front_disk_position (img)
    rear_disk_position =  get_back_disk_position (img)
    orientation = np.arctan2(rear_disk_position[1]-front_disk_position[1] , rear_disk_position[0]-front_disk_position[0])
    return orientation
orient = get_robot_orientation(img)
print(orient)


def get_robot_position(img):
    front_disk_position = get_front_disk_position (img)
    rear_disk_position =  get_back_disk_position (img)
    robot_pos = (front_disk_position+rear_disk_position)/2
    results,Pioneer = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx",vrep.simx_opmode_blocking)
    results,Position=vrep.simxGetObjectPosition(clientID,Pioneer,-1,vrep.simx_opmode_streaming)  
    results,Position=vrep.simxGetObjectPosition(clientID,Pioneer,-1,vrep.simx_opmode_buffer)
    print(Position)
    return tuple(robot_pos.astype('int'))

center_robot = get_robot_position(img)
print(center_robot)

plt.imshow(img)
plt.plot(center_robot[1],center_robot[0], "*r")
plt.show()


def get_goal_position(img):
    goal_mask = cv2.inRange(img[:,:,1], 240, 255)
    goal_mask = cv2.GaussianBlur(goal_mask, (9, 9), 0)
    contours, hierarchy = cv2.findContours(goal_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #no need for the for loop but it is used to avoid errors
    for c in contours:
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

    return goal_mask, (cY,cX)

#
img_goal, center_goal = get_goal_position(img)

plt.imshow(img_goal, cmap="gray")
plt.show()

#Xtarget = input("Enter Target x Position :")
#Xtarget = int(Xtarget)
#Ytarget = input("Enter Target y Position :")
#Ytarget = int(Ytarget)
#center_goal = tuple(np.array([Xtarget,Ytarget]))
print(center_goal)


def search(grid,init,goal, cost):
    visit = [[False for _ in range(512)] for _ in range(512)]
    queue =[(0, init)]
   
    def delta_gain(gain = 1):
        delta = np.array([[-1, 0], # go up
                          [-1,-1], # up left
                          [ 0,-1], # go left
                          [ 1,-1], # down left
                          [ 1, 0], # go down
                          [ 1, 1],  # down right
                          [ 0, 1], # go right         
                          [-1, 1] # up right         
         ]) 
        return delta*gain
    delta = delta_gain(gain = 5)
    discovered = []
    discovered.append(init)
    actions = np.ones_like(grid)*-1
    count = 0
    path = []
    visit[init[0]][init[1]]= True
    def near_obstacles(point, half_kernel = 15):
        x_start = int(max(point[0] - half_kernel, 0))
        x_end = int(min(point[0] + half_kernel, grid.shape[0]))
        y_start = int(max(point[1] - half_kernel, 0))
        y_end = int(min(point[1] + half_kernel, grid.shape[1]))
        return np.any(grid[x_start:x_end, y_start:y_end]<128) or x_start<5 or x_end>495 or y_start<5 or y_end>495
    def policy_draw(indx):
        indx_old = tuple(indx)
        indx_new = tuple(indx)
        path.append(tuple(goal))
        while indx_new != init:
            indx_new = tuple( np.array(indx_old) - delta[int(actions[indx_old])] )
            path.append(indx_new)
            indx_old = indx_new
    while queue:
        dist, indx = heapq.heappop(queue)
        i, j = indx[0], indx[1]
        if ((indx[0] >= goal[0]-20) and (indx[0] < goal[0]+20)) and ((indx[1] >= goal[1]-20) and (indx[1] < goal[1]+20)):
            policy_draw(indx)
            print("found goal")
            print(count)
            return actions, np.array(path[::-1])
        else:
            for y in range(len(delta)) :
                indx_new = tuple(indx + delta[y])
                if ((np.any(np.array(indx_new) < 0)) or (indx_new[0] > grid.shape[0]-1) or (indx_new[1] > grid.shape[1]-1)) :
                    continue
                if (grid[indx_new] >= 128) and (not visit[indx_new[0]][indx_new[1]]):
                    count += 1
                    # if the obstacle is inside the robot :D, have a really high cost
                    if near_obstacles(indx_new, half_kernel = 15):
                        dist_new = dist + 15000*cost

                    # if the obstacle is about a robot's length near it , have a high cost
                    elif near_obstacles(indx_new, half_kernel = 25):
                        dist_new = dist + 5000*cost
                    # as before
                    else:
                        dist_new = dist + cost
        
                    #trying to increase the cost of rapidly changing direction
                    
                    if y == actions[indx]:
                        dist_new = dist_new 
                    elif (y-1)%len(delta) == actions[indx] or (y+1)%len(delta) == actions[indx]:
                        dist_new = dist_new + 5*cost
                    else :
                        dist_new = dist_new + 10*cost
                    
                    heapq.heappush(queue, (dist_new, indx_new))
                    i, j = indx_new[0], indx_new[1]
                    visit[i][j] =True
                    actions[indx_new] = y     
    
    print(count)
    print("fail")



tick = time.time()
actions, path = search(img_obs,center_robot,center_goal,cost = 1)
tock = time.time()
print(tock-tick)
print(img_obs.shape)
print(path)
print(np.max(actions))

newpath = path
def transform_points_from_image2real (points):
    if points.ndim < 2:
        flipped = np.flipud(points)
    else:
        flipped = np.fliplr(points)
    scale = 5/445
    points2send = (flipped*-scale) + np.array([2.0555+0.75280899, -2.0500+4.96629213])
    return points2send


def transform2robot_frame(pos, point, theta):
    pos = np.asarray(pos)
    point = np.asarray(point)
    T_matrix = np.array([
            [np.sin(theta), np.cos(theta)],
            [np.cos(theta), -1*np.sin(theta)],
            ])
    trans = point-pos
    if trans.ndim >= 2:
        trans = trans.T
        point_t = np.dot(T_matrix, trans).T
    else:
        point_t = np.dot(T_matrix, trans)
    return point_t


def is_near(robot_center, point, dist_thresh = 0.25):
    dist = np.sqrt((robot_center[0]-point[0])**2 + (robot_center[1]-point[1])**2)
    return dist<=dist_thresh


def get_distance(points1, points2):
    return np.sqrt(np.sum(np.square(points1 - points2), axis=1))

'''

'''  
    
def send_path_4_drawing(path, sleep_time = 0.07):
    #the bigger the sleep time the more accurate the points are placed but you have to be very patient :D
    for i in path:
        point2send = transform_points_from_image2real (i)
        packedData=vrep.simxPackFloats(point2send.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)	
        returnCode=vrep.simxWriteStringStream(clientID, "path_coord", raw_bytes, vrep.simx_opmode_oneshot)
        time.sleep(sleep_time)
 
d = 0.381 #wheel axis distance
r_w = 0.195/2


def pioneer_robot_model(v_des, omega_des):
    v_r = (v_des+d*omega_des)
    v_l = (v_des-d*omega_des)
    omega_right = v_r/r_w
    omega_left = v_l/r_w
    return omega_right, omega_left

lad = 0.5 #look ahead distance
print ('Starting Connection')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
print(clientID)

if clientID!=-1:
    print ('Connected to remote API server')  
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)
    print('start',e)
    try:
        res,camera0_handle = vrep.simxGetObjectHandle(clientID,'top_view_camera',vrep.simx_opmode_oneshot_wait)
        res_l,right_motor_handle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
        res_r,left_motor_handle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
        res_las,look_ahead_sphere = vrep.simxGetObjectHandle(clientID,'look_ahead',vrep.simx_opmode_oneshot_wait)
        res_las,goal = vrep.simxGetObjectHandle(clientID,'goal',vrep.simx_opmode_oneshot_wait)
        
        indx = 0
        theta = 0.0
        dt = 0.0
        count = 0
        om_sp = 0
        d_controller   = PID(0.4, 0.0, 0.05)
        omega_controller = PID(0.6, 0.0, 0.05)
        path_to_track = transform_points_from_image2real(path)
        send_path_4_drawing(path, 0.05)
        center_goal = transform_points_from_image2real(np.array(center_goal))
        result = vrep.simxSetObjectPosition(clientID,goal,-1,center_goal,vrep.simx_opmode_oneshot)
        while not is_near(center_robot, center_goal, dist_thresh = 0.25):            
            tick = time.time()
            err,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera0_handle,0,vrep.simx_opmode_streaming)        
            if err == vrep.simx_return_ok:
                img = np.array(image,dtype=np.uint8)
                img.resize([resolution[1],resolution[0],3])
                center_robot = get_robot_position(img)
                center_robot = transform_points_from_image2real(np.array(center_robot))
                theta = get_robot_orientation(img)
                theta = np.arctan2(np.sin(theta), np.cos(theta))
                path_transformed = transform2robot_frame(center_robot, path_to_track, theta)
                dist = get_distance(path_transformed, np.array([0,0]))
                #loop to determine which point will be the carrot
                for i in range(dist.argmin(), dist.shape[0]):
                    if dist[i] < lad and indx <= i:
                        indx = i
                #mark the carrot with the sphere
                returnCode=vrep.simxSetObjectPosition(clientID,look_ahead_sphere,-1,(path_to_track[indx,0], path_to_track[indx,1], 0.005),vrep.simx_opmode_oneshot)
                orient_error = np.arctan2(path_transformed[indx,1], path_transformed[indx,0])
                #the controllers 
                v_sp = d_controller.update(dist[indx]) 
                om_sp = omega_controller.update(orient_error)
                vr, vl = pioneer_robot_model(v_sp, om_sp)
                errorCode_leftM = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, vr, vrep.simx_opmode_oneshot)
                errorCode_rightM = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle,vl, vrep.simx_opmode_oneshot)
                count += 1
                tock = time.time()                
                dt = tock - tick
                print(dt)
        else:
            print("GOAAAAAAALL !!")
    finally:
        time.sleep(0.1)
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
        vrep.simxFinish(-1)
