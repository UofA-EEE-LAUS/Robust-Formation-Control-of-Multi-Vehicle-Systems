%MATLAB script for controlling the 3 omni-wheel rover platform

%---------------------------------SCRIPT----------------------------------%
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%Formation Setting
u = [2;0;4;0;4];
v = [2;0;4;4;0];


%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    elapsedTime = 1;
    
    %------------------------------CODE HERE------------------------------%
    
    %Get 5 rovers' position
    
    [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover'),vrep.simx_opmode_blocking);
    [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover#0'),vrep.simx_opmode_blocking);
    [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover#1'),vrep.simx_opmode_blocking);
    [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover#2'),vrep.simx_opmode_blocking);
    [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover#3'),vrep.simx_opmode_blocking);
    [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
    
    position = [position_0; position_1; position_2; position_3; position_4];
    
    %Get formation
    [destination, roverID] = formation_5_LUA(position(:,1),position(:,2),u,v);
    
    %Running rovers
    a=[destination(roverID(1),1),destination(roverID(1),2),180,destination(roverID(2),1),destination(roverID(2),2),180,destination(roverID(3),1),destination(roverID(3),2),0,destination(roverID(4),1),destination(roverID(4),2),0,destination(roverID(5),1),destination(roverID(5),2),0];
    packedData=vrep.simxPackFloats(a);
    [returnCode]=vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot);
    
else
    disp('Failed connecting to remote API server');
end

vrep.delete()