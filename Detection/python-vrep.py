#py 3.7 and Spyder 4.1.2
# import math
import time
import numpy as np
import sim as vrep
import cv2
from framebyframe import frame_buffer
from ATF import alphatrimmer
from support import info_frame
from CF import ComplementaryFilter
from kalman_filter import kalman_filter

#Setting up the API
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

#setting up font, colors for display and extra dummy variables
frame_id = 0
FONT = cv2.FONT_HERSHEY_PLAIN
RPY = [0,0,0]                               

#Setting up Lidar, gyro, Mag, vision sensor
#Check Remote API for function calls and operation modes
ERRORCODE, Lidar = vrep.simxGetObjectHandle(clientID, 'laser_sensor#2', vrep.simx_opmode_blocking)
#Using absoulte position of the rover as a stand in for Magnetometer
ERRORCODE, Magref = vrep.simxGetObjectHandle(clientID,'rover#2',vrep.simx_opmode_blocking)
res, v1 = vrep.simxGetObjectHandle(clientID, 'Vision_sensor0#2', vrep.simx_opmode_oneshot_wait)
ERRORCODE, Rover3 = vrep.simxGetObjectHandle(clientID, 'rover#3', vrep.simx_opmode_blocking)



if clientID != -1:
    starting_time = time.time()
    #Setting up Lidar, gyro, Mag, vision sensor for first time detection, simx_opmode_streaming is first call as mentioned in Remote API
    ERRORCODE, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    ERRORCODE, detectionState, detectedPoint_lidar, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, Lidar, vrep.simx_opmode_streaming)
    ERRORCODE, eulerAngles=vrep.simxGetObjectOrientation( clientID, Magref, -1, vrep.simx_opmode_streaming)
    ERRORCODE, gyro_out = vrep.simxGetStringSignal(clientID, "myGyroData#2", vrep.simx_opmode_streaming)
    ERRORCODE, accel_out = vrep.simxGetStringSignal(clientID, "myaccData#2", vrep.simx_opmode_streaming)
    ERRORCODE, position = vrep.simxGetObjectPosition( clientID, Magref, -1, vrep.simx_opmode_streaming)
    ERRORCODE, position2 = vrep.simxGetObjectPosition( clientID, Rover3, -1, vrep.simx_opmode_streaming)




    while vrep.simxGetConnectionId(clientID) != -1:
        #Setting up Lidar, gyro, Mag, vision sensor for detection values from buffer, simx_opmode_buffer is for all subsequent calls after first call as mentioned in Remote API        
        ERRORCODE, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        ERRORCODE, detectionState, detectedPoint_lidar, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, Lidar, vrep.simx_opmode_buffer)
        # ERRORCODE, gyro_data = vrep.simxGetFloatSignal(clientID, gyroRef, vrep.simx_opmode_buffer)
        ERRORCODE, eulerAngles=vrep.simxGetObjectOrientation( clientID, Magref, -1, vrep.simx_opmode_buffer)        
        ERRORCODE, gyro_out = vrep.simxGetStringSignal( clientID, "myGyroData#2", vrep.simx_opmode_buffer)
        ERRORCODE, accel_out = vrep.simxGetStringSignal( clientID, "myaccData#2", vrep.simx_opmode_buffer)
        ERRORCODE, position = vrep.simxGetObjectPosition( clientID, Magref, -1, vrep.simx_opmode_buffer)
        ERRORCODE, position2 = vrep.simxGetObjectPosition( clientID, Rover3, -1, vrep.simx_opmode_buffer)
        
        #linearizing detected Lidar points
        norm_distance = np.linalg.norm(detectedPoint_lidar)
        
        #parsing data to alpha trimmmer
        filtered_data = alphatrimmer(7, 2, str(norm_distance))
        
        frame_id += 1

        if ERRORCODE == vrep.simx_return_ok:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[1], resolution[0], 3])
            vision_sensor_output = cv2.rotate(img, cv2.ROTATE_180)
            try:
                parsing_data = frame_buffer(vision_sensor_output)
                elapsed_time = time.time() - starting_time
                fps = frame_id / elapsed_time
                #Unpacking float datas from gyro and accel
                floatValues1 = vrep.simxUnpackFloats( gyro_out)
                floatValues2 = vrep.simxUnpackFloats( accel_out)
                
                #parsing data to alpha trimmmer
                CF_out = ComplementaryFilter(floatValues2, floatValues1, eulerAngles, RPY)
                
                #parsing data to alpha trimmmer
                KF_Out = kalman_filter(filtered_data, 0, 0, 0) 
                    
                display_frame = info_frame(parsing_data, filtered_data, fps, KF_Out, CF_out[2], position, position2)
                cv2.imshow("Image", display_frame)
                # print("Detection successful")
                cv2.waitKey(100)
            except:
                cv2.imshow("Image", vision_sensor_output)

        elif ERRORCODE == vrep.simx_return_novalue_flag:
            pass
        else:
            print(ERRORCODE)
else:
    vrep.simxFinish(clientID)
cv2.destroyAllWindows()

