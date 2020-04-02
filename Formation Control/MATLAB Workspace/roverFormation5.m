%MATLAB script for controlling the 3 omni-wheel rover platform

%---------------------------------SCRIPT----------------------------------%
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%Formation Setting
u = [0;-2;-2;2;2];
v = [0;-2;2;2;-2];


%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    elapsedTime = 1;
    
    %------------------------------CODE HERE------------------------------%
    
    %Get 5 rovers' position
    
    [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
    [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover_1'),vrep.simx_opmode_blocking);
    [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover_2'),vrep.simx_opmode_blocking);
    [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover_3'),vrep.simx_opmode_blocking);
    [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover_4'),vrep.simx_opmode_blocking);
    [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
    
    position = [position_0; position_1; position_2; position_3; position_4];
        
    position_r0 = position_0;
    position_r1 = position_1;
    position_r2 = position_2;
    position_r3 = position_3;
    position_r4 = position_4;
    
    flag = zeros(5,1);
    for loop = 1:6
        
        tic
        
        [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
        [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
        [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover_1'),vrep.simx_opmode_blocking);
        [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
        [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover_2'),vrep.simx_opmode_blocking);
        [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
        [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover_3'),vrep.simx_opmode_blocking);
        [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
        [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover_4'),vrep.simx_opmode_blocking);
        [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
        
        position = [position_0; position_1; position_2; position_3; position_4];
        
        [destination, roverID] = formation_5(position(:,1),position(:,2),u,v);
        
        %Run rovers
        [position_r0] = locate(destination(1,1),destination(1,2),position_r0,elapsedTime,int2str(roverID(1)),vrep,clientID);
        [position_r1] = locate(destination(2,1),destination(2,2),position_r1,elapsedTime,int2str(roverID(2)),vrep,clientID);
        [position_r2] = locate(destination(3,1),destination(3,2),position_r2,elapsedTime,int2str(roverID(3)),vrep,clientID);
        [position_r3] = locate(destination(4,1),destination(4,2),position_r3,elapsedTime,int2str(roverID(4)),vrep,clientID);
        [position_r4] = locate(destination(5,1),destination(5,2),position_r4,elapsedTime,int2str(roverID(5)),vrep,clientID);
        
        %         pause(1)
        
        elapsedTime = toc;
        
        
    end
    
    %destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end

vrep.delete()