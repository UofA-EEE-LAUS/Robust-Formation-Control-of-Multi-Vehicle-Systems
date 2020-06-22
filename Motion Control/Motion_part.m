% The university of Adelaide
% Kamalpreet singh
% 2019
% Instructions
% this matlab function is used to build a connection with the Vrep(simulation software) 
% it is a prototype that tells us the way the desired points are fed into the 
% simulation software.
% this matlab file should be in the folder having the following files
% 1 remApi.m
% 2 remoteApiProto.m
% 3 remote API library in windows -remoteApi.dll or can be found in the 
% library folder of the matlab programming in the local drive C.
% 4 simpleTest.m
% starting the function
clear();
vrep=remApi('remoteApi');% storing the remoteApi in the vrep variable
vrep.simxFinish(-1);% finishing any ongoing simulations
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);% connection line
if (clientID>-1)
    disp('Connected')
   vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);% simulation start in opmode status
flag =false;
[returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
 % starting while loop
while 1
   a=input('Value of x = '); % input the vector in '[x_n y_n theta_n]' these are the x y theta for all the rovers.
     packedData=vrep.simxPackFloats(a);% packing the data that was input for a
     [returnCode]=vrep.simxWriteStringStream(clientID,'signal',packedData,vrep.simx_opmode_oneshot);% writing the data into signal stream named signal 
end
 vrep.simxFinish(-1);% finishing the simulation
end
% deleting the function
vrep.delete();