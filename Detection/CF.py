'''
Complementary Function for IMU, low pass filtering for accelerometer, high pass filtering for gyroscope
'''

import math

#Dt denotes the number of samples to consider per second
dt = 1/10 #0.1 readings per second
RPY = [0,0,0]                               




def ComplementaryFilter(Accel_data, gyro_data, Mag_data, RPYaw):
    '''
    Parameters
    ----------
    Accel_data : Float array
        DESCRIPTION: Accelrometer data from Remote API
    gyro_data : Float array
        DESCRIPTION: Gyroscope data from Remote API
    Mag_data : Float array
        DESCRIPTION: Magnetometer data from Remote API
    RPYaw : empty array
        DESCRIPTION: Placeholder for passing previous data or dummy data

    Returns
    -------
    RPY : 1x3 array
        DESCRIPTION: Roll, Pitch and Yaw data
    '''
    #Integrating the gyro data
    RPYaw[0] += gyro_data[0]*dt
    RPYaw[1] -= gyro_data[1]*dt
    
    #Calulating Row and Pitch
    rollXL = math.degrees(math.atan2(Accel_data[1], Accel_data[2]))
    pitchXL = math.degrees(math.atan2(-1*Accel_data[0], math.sqrt(Accel_data[1]**2+Accel_data[2]**2)))

    #Getting the yaw angle from the Magnetometer, Since Magnetometer is not designed in CoppeliaSim, we can use self refrence as an alternative as the forum suggests
    MagYaw = math.degrees(math.atan2(-1*(Mag_data[1]), (Mag_data[0])))
   
    #The percentage of Gyro data vs Accelerometer data
    ratio = 0.50    
    RPYaw[0] = RPYaw[0]*ratio + rollXL*(1-ratio)    #applying a low pass filter to the accelerometer
    RPYaw[1] = RPYaw[1]*ratio + pitchXL*(1-ratio)   #applying a high pass filter to the gyroscope
    
    #Calculating the Yaw from the Magnetometer data 
    RPYaw[2] = MagYaw
    
    RPY[0]= RPYaw[0] #Roll
    RPY[1] = RPYaw[1] #Pitch
    RPY[2] = RPYaw[2] #Yaw
    
    return RPY
