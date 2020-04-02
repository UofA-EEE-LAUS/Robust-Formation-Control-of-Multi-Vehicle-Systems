clear();
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID>-1)
    disp('Connected')
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    flag = false;
    a=[-2 4 60 -1 3];
    [returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
    
    while 1
        a=input('Value of x = ');
        packedData=vrep.simxPackFloats(a);
        [returnCode]=vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot);
        
    end
    vrep.simxFinish(-1);
end

vrep.delete();