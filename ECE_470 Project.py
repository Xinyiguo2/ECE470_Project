import vrep     
import sys
import time                
import numpy as np         
import math
import cv2 as opencv
 

PI=math.pi  

#Initialization
vrep.simxFinish(-1) 

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  
    print ('Connected to remote API server')
    
else:
    print ('Connection not successful')
    sys.exit('Could not connect')


#def get_handles
# Robot Handles
results,Pioneer = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx",vrep.simx_opmode_blocking)
 
     
# Motor Handles

result,left_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
result,right_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)


sensor_h=[] 
sensor_val=np.array([]) 




sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

#assign handle for each proximity sensor
for x in range(1,16+1):
        result,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x),vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle)      
        result,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) 
        
t = time.time()

# Target Position
Xtarget = input("Enter Target x Position :")
Xtarget = float(Xtarget)
Ytarget = input("Enter Target y Position :")
Ytarget = float(Ytarget)
Target = np.array([Xtarget,Ytarget])



##
V_left = 1
V_right = 1



while (time.time()-t)<60:
    sensor_val=np.array([])
    ## Get robot heading and position, Heading is R_z, or Heading[2]
    results,Heading=vrep.simxGetObjectOrientation(clientID,Pioneer,-1,simx_opmode_streaming)
    results,Position=simxGetObjectPosition(clientID,Pioneer,-1,simx_opmode_streaming)  
    results,Heading=vrep.simxGetObjectOrientation(clientID,Pioneer,-1,simx_opmode_buffer)
    results,Position=simxGetObjectPosition(clientID,Pioneer,-1,simx_opmode_buffer)    
    ## Proximity Sensor Readings
    for x in range(1,16+1):
        result,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x-1],vrep.simx_opmode_buffer)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) 
    
    sensor_sq=sensor_val[0:8]*sensor_val[0:8] 
        
    min_ind=np.where(sensor_sq==np.min(sensor_sq))
    min_ind=min_ind[0][0]
    
#    obstacle distance vector
    if sensor_sq[min_ind]<0.15:
        steer=-1/sensor_loc[min_ind]*sensor_sq[min_ind]
    else:
        steer=0
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#    Robot Kinematics
    def Robot_Motion_Model(V,W):
        B = .381  #Wheel Base
        R = 0.195 #Wheel Radius
        V_left = V + R*W
        V_right = V - R*W
        W_L = V_left/R
        W_R = V_right/R
        return W_L, W_R
   
    
        
    
    
    

    result=vrep.simxSetJointTargetVeloc ity(clientID,left_motor_handle,W_left, vrep.simx_opmode_streaming)
    result=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle, W_right, vrep.simx_opmode_streaming)

    time.sleep(0.2) 

result=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
result=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    

