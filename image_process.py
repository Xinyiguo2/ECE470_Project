#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 17 01:22:03 2019

@author: yihuicui
"""
import cv2
import numpy as np
import sys
from queue import PriorityQueue
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
from PIL import Image



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
        err = 10
        res,camera0_handle = vrep.simxGetObjectHandle(clientID,'top_view_camera',vrep.simx_opmode_oneshot_wait)
       
        while err != vrep.simx_return_ok:
            tick = time.time()
            err,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera0_handle,0,vrep.simx_opmode_streaming)  
            if err == vrep.simx_return_ok:
                img = np.array(image,dtype=np.uint8)
                img.resize([resolution[1],resolution[0],3])
                im = Image.fromarray(img)
                im.save("top_view.png")
    finally:
        time.sleep(0.1)
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
        vrep.simxFinish(-1)

