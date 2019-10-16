import vrep
import time
import numpy as np
import cv2 as opencv
import matplotlib.pyplot as plt 
from math import cos, sin
from scipy.linalg import expm,logm


# Get distances measurements from each joint center to base frame (useful for forward kinematics)
#def get_joint():
#	X = []
#	Y = []
#	Z = []
#	result,vector=vrep.simxGetObjectPosition(clientID, joint_one_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	result,vector=vrep.simxGetObjectPosition(clientID, joint_two_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	result,vector=vrep.simxGetObjectPosition(clientID, joint_three_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	result,vector=vrep.simxGetObjectPosition(clientID, joint_four_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	result,vector=vrep.simxGetObjectPosition(clientID, joint_five_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	result,vector=vrep.simxGetObjectPosition(clientID, joint_six_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
#	X.append(vector[0])
#	Y.append(vector[1])
#	Z.append(vector[2])
#	X = np.round(X, decimals = 3)
#	Y = np.round(Y, decimals = 3)
#	Z = np.round(Z, decimals = 3)
#	return X,Y,Z

# Function that used to move joints
#def SetJointPosition(theta):
#	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
#	time.sleep(0.5)
#	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
#	time.sleep(0.5)
#	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
#	time.sleep(0.5)
#	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
#	time.sleep(0.5)
#	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
#	time.sleep(0.5)
#	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
#	time.sleep(0.5)

# Function that used to read joint angles
#def GetJointAngle():
#	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
#	if result != vrep.simx_return_ok:
#		raise Exception('could not get 1 joint variable')
#	result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
#	if result != vrep.simx_return_ok:
#		raise Exception('could not get 2 joint variable')
#	result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
#	if result != vrep.simx_return_ok:
#		raise Exception('could not get 3 joint variable')
#	result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
#	if result != vrep.simx_return_ok:
#		raise Exception('could noconda


# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

'''
# Print object name list
result,joint_name,intData,floatData,stringData = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking)
print(stringData)
'''
# Get "handle" to the base of robot
result, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for left Motor')
result, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking  )
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for Right Motor')
result, sensor1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', vrep.simx_opmode_blocking  )
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for Sensor1')
result, cam_handle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for Camera')
result, cam_handle_2 = vrep.simxGetObjectHandle(clientID, 'DefaultCamera', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for Camera')
 
 
## ==================================================================================================== #
#
## Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
#
## ******************************** Your robot control code goes here  ******************************** #
time.sleep(1)
vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,-5,vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,-5 ,vrep.simx_opmode_streaming)

result,detectionState, detectedPoint, detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor1,vrep.simx_opmode_streaming)

result,detectionState, detectedPoint, detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor1,vrep.simx_opmode_buffer)

result,resolution,image = vrep.simxGetVisionSensorImage(clientID,cam_handle_2,0,vrep.simx_opmode_streaming)
#result,resolution,image = vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_buffer)

##
##Goal_joint_angles = np.array([[0,0,-0.5*np.pi,0.5*np.pi,-0.5*np.pi,-0.5*np.pi], \
##							[0.5*np.pi,0,-0.5*np.pi,0.5*np.pi,0.5*np.pi,-0.5*np.pi],\
##							[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,-0.5*np.pi]])
##for i in range(3):
##
##        SetJointPosition(Goal_joint_angles[i])
#    

# Wait two seconds
time.sleep(2)
# **************************************************************************************************** #

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)


print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #