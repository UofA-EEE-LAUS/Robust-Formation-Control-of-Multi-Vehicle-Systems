%MATLAB script for controlling the 3 omni-wheel rover platform

%---------------------------------SCRIPT----------------------------------%
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    elapsedTime = 1;
    threshold = 0.3;
    
    %------------------------------CODE HERE------------------------------%
    
    %Get 3 rovers' position
    
    [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
    [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
    
    position_r0 = position_0;
    
    x = 5;
    y = 5;
    
    while (abs(position_r0(1,1)-x) >= threshold || abs(position_r0(1,2)-y) >= threshold)
        tic
        
        [position_r0] = locate (x,y,position_r0,elapsedTime,'0',vrep,clientID);
        
        elapsedTime = toc;
    end
    
    
    
    %destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end

vrep.delete()