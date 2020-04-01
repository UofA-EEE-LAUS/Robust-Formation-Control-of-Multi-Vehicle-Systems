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
    
    %Get 5 obstacles' position
    [returnCode,Cuboid11]=vrep.simxGetObjectHandle(clientID,strcat('Cuboid11'),vrep.simx_opmode_blocking);
    [returnCode,cuboid_1]=vrep.simxGetObjectPosition(clientID,Cuboid11,-1,vrep.simx_opmode_blocking);
    [returnCode,Cuboid12]=vrep.simxGetObjectHandle(clientID,strcat('Cuboid12'),vrep.simx_opmode_blocking);
    [returnCode,cuboid_2]=vrep.simxGetObjectPosition(clientID,Cuboid12,-1,vrep.simx_opmode_blocking);
    [returnCode,Cuboid13]=vrep.simxGetObjectHandle(clientID,strcat('Cuboid13'),vrep.simx_opmode_blocking);
    [returnCode,cuboid_3]=vrep.simxGetObjectPosition(clientID,Cuboid13,-1,vrep.simx_opmode_blocking);
    [returnCode,Cuboid14]=vrep.simxGetObjectHandle(clientID,strcat('Cuboid14'),vrep.simx_opmode_blocking);
    [returnCode,cuboid_4]=vrep.simxGetObjectPosition(clientID,Cuboid14,-1,vrep.simx_opmode_blocking);
    [returnCode,Cuboid15]=vrep.simxGetObjectHandle(clientID,strcat('Cuboid15'),vrep.simx_opmode_blocking);
    [returnCode,cuboid_5]=vrep.simxGetObjectPosition(clientID,Cuboid15,-1,vrep.simx_opmode_blocking);
    
    %Get formation
    [destination, roverID] = formation_5_LUA(position(:,1),position(:,2),u,v);
    
    %obstacle (raw for No., column for x/y coordinate)
    obstacle = [cuboid_1(1), cuboid_2(1), cuboid_3(1),cuboid_4(1), cuboid_5(1);
        cuboid_1(2), cuboid_2(2), cuboid_3(2), cuboid_4(2), cuboid_5(2)];
    obstacle_1 = [obstacle(1,:), position([2 3 4 5],1)';
        obstacle(2,:), position([2 3 4 5],2)'];
    obstacle_2 = [obstacle(1,:), position([1 3 4 5],1)';
        obstacle(2,:), position([1 3 4 5],2)'];
    obstacle_3 = [obstacle(1,:), position([1 2 4 5],1)';
        obstacle(2,:), position([1 2 4 5],2)'];
    obstacle_4 = [obstacle(1,:), position([1 2 3 5],1)';
        obstacle(2,:), position([1 2 3 5],2)'];
    obstacle_5 = [obstacle(1,:), position([1 2 3 4],1)';
        obstacle(2,:), position([1 2 3 4],2)'];
    
    
    
    
    for i = 1:20
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
        
        %obstacle (raw for No., column for x/y coordinate)
        obstacle_1 = [obstacle(1,:), position([2 3 4 5],1)';
            obstacle(2,:), position([2 3 4 5],2)'];
        obstacle_2 = [obstacle(1,:), position([1 3 4 5],1)';
            obstacle(2,:), position([1 3 4 5],2)'];
        obstacle_3 = [obstacle(1,:), position([1 2 4 5],1)';
            obstacle(2,:), position([1 2 4 5],2)'];
        obstacle_4 = [obstacle(1,:), position([1 2 3 5],1)';
            obstacle(2,:), position([1 2 3 5],2)'];
        obstacle_5 = [obstacle(1,:), position([1 2 3 4],1)';
            obstacle(2,:), position([1 2 3 4],2)'];
        
        %Calculate Path
        %rover 1
        path_1 = Path([position_0(1);position_0(2)],[destination(roverID(1),1);destination(roverID(1),2)],obstacle_1);
        %rover 2
        path_2 = Path([position_1(1);position_1(2)],[destination(roverID(2),1);destination(roverID(2),2)],obstacle_2);
        %rover 3
        path_3 = Path([position_2(1);position_2(2)],[destination(roverID(3),1);destination(roverID(3),2)],obstacle_3);
        %rover 4
        path_4 = Path([position_3(1);position_3(2)],[destination(roverID(4),1);destination(roverID(4),2)],obstacle_4);
        %rover 5
        path_5 = Path([position_4(1);position_4(2)],[destination(roverID(5),1);destination(roverID(5),2)],obstacle_5);
        
        %Running rovers
        a=[path_1(1,5),path_1(2,5),0,path_2(1,5),path_2(2,5),0,path_3(1,5),path_3(2,5),0,path_4(1,5),path_4(2,5),0,path_5(1,5),path_5(2,5),0];
        packedData=vrep.simxPackFloats(a);
        [returnCode]=vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot);
        
    end
    
    
    
else
    disp('Failed connecting to remote API server');
end

vrep.delete()