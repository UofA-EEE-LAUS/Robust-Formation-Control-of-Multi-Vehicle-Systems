function [] = Move (x,y)


    
    %constructs remote api object
    vrep=remApi('remoteApi');
    %destroy's any current connections to v-rep simulation
    vrep.simxFinish(-1);
    %establishes connection to v-rep simulation on port 19999
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    
    
    rover_radius = 15;
    wheel_radius = 5.22;
    constant_linearVelocity = (0.5*pi*wheel_radius);
    phi = 0/180*pi;
    dphi = 0.5/180*pi;
    
    %get position
    
    [returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
    position_x=position(:,1);
    position_y=position(:,2);

    %start moving
    while abs(position_x-x) >=0.2 || abs(position_y-y) >= 0.2
        
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
        position_x=position(:,1);
        position_y=position(:,2);
        
        dx = x - position_x;
        dy = y - position_y;
        time_limit = sqrt(dx*dx + dy*dy)/constant_linearVelocity;
  
        v_x = dx / time_limit;
        v_y = dy / time_limit;
        w = dphi / time_limit;
  
        v   =  v_x * cos(phi) + v_y * sin(phi);
        v_n = -v_x * sin(phi) + v_y * cos(phi);
  
        v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
        v1 =                - v_n             + w * rover_radius;
        v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;

        %defining motor handles
        %return code functions as a debug tool/error message
        [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,'motor_0',vrep.simx_opmode_blocking);
        [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,'motor_1',vrep.simx_opmode_blocking);
        [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,'motor_2',vrep.simx_opmode_blocking);
    
        %setting motor speeds for straight line
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,v0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,v1,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,v2,vrep.simx_opmode_blocking);
        
    end
    
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
end
